use ordered_float::OrderedFloat;
use std::{
    collections::HashSet,
    ffi::{c_void, CStr},
};
use survsim_structs::{
    backend::Task,
    plan::{Dependencies, Plan, PlanTask},
    problem::Problem,
    report::Location,
};
use tinyvec::TinyVec;

use crate::{
    shortest_path::plan_vehicle,
    txgraph::{self, production_edge, Node},
};

#[test]
pub fn test() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../tiny_problem.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    println!("{:?}", x);
}

pub fn solve(problem: &Problem) -> Plan {
    HeuristicColgenSolver::new(problem).solve_heuristic()
}

#[derive(Debug)]
struct BattCycPlan {
    cost: f32,
    path: Vec<u32>,
}

struct HeuristicColgenSolver<'a> {
    #[allow(unused)]
    problem: &'a Problem,
    vehicle_start_nodes: Vec<(u32, f32)>,
    nodes: Vec<Node>,
    time_steps: Vec<i32>,
    time_steps_vehicles: Vec<u32>,
    columns: Vec<BattCycPlan>,
    battcycplans: Vec<BattCycPlan>,
    label_buf: Vec<TinyVec<[crate::shortest_path::Label; 20]>>,
    idxs_buf: Vec<i32>,
    coeffs_buf: Vec<f64>,
}

fn convert_batt_cyc_plan(
    problem: &Problem,
    nodes: &[Node],
    vehicle_start_nodes: &[(u32, f32)],
    mut components: Vec<BattCycPlan>,
) -> Plan {
    // Sort components by time
    components.sort_by_key(|bcp| nodes[bcp.path[0] as usize].state.time);

    let mut vehicles_ready_at_base: tinyvec::TinyVec<[(u32, i32); 10]> = problem
        .vehicles
        .iter()
        .enumerate()
        .filter_map(|(i, v)| Some((i as u32, 0i32))) // TODO check if vehicle is at base at start
        .collect();

    let mut v_plans = vec![
        vec![PlanTask {
            task: Task::Wait,
            dependencies: Default::default()
        }];
        problem.vehicles.len()
    ];

    for plan in components {
        let v_init = match nodes[plan.path[0] as usize].state.loc {
            Location::DroneInitial(d) => Some(d),
            _ => None,
        };
        let start_path_idx = plan
            .path
            .iter()
            .zip(plan.path.iter().skip(1))
            .position(|(n1, n2)| {
                let (s1, s2) = (&nodes[*n1 as usize].state, &nodes[*n2 as usize].state);
                let on_ground = (s1.loc == Location::Base && s2.loc == Location::Base)
                    || (s1.loc == Location::SinkNode && s2.loc == Location::SinkNode);
                !on_ground
            })
            .unwrap();

        let end_path_idx = plan
            .path
            .iter()
            .position(|node_idx| nodes[*node_idx as usize].state.loc == Location::SinkNode)
            .unwrap();

        let start_time = nodes[plan.path[start_path_idx] as usize].state.time;
        let end_time = nodes[plan.path[end_path_idx] as usize].state.time;

        let v_idx = if let Some(v_idx) = v_init {
            vehicles_ready_at_base.push((v_idx as u32, end_time));
            v_idx
        } else {
            let ready_idx = vehicles_ready_at_base
                .iter()
                .position(|(_, t)| *t <= start_time)
                .unwrap();
            vehicles_ready_at_base[ready_idx].1 = end_time;
            vehicles_ready_at_base[ready_idx].0 as usize
        };

        let vplan = &mut v_plans[v_idx];

        for (n1, n2) in plan.path[start_path_idx..end_path_idx]
            .iter()
            .zip(plan.path[start_path_idx + 1..].iter())
        {
            let (s1, s2) = (&nodes[*n1 as usize], &nodes[*n2 as usize]);
            assert!(s1.state.time < s2.state.time);
            match (s1.state.loc, s2.state.loc) {
                (_, Location::Base)
                | (_, Location::DroneInitial(_))
                | (Location::SinkNode, _)
                | (Location::Base, Location::SinkNode) => {
                    panic!()
                }
                (Location::Base, Location::Task(task)) => {
                    // vplan.append(PlanTask { task: Task::GotoPoi(task), dependencies: Dependencies::wait(t)});
                }
                (Location::DroneInitial(_), Location::Task(task_ref)) => todo!(),
                (Location::DroneInitial(_), Location::SinkNode) => todo!(),
                (Location::Task(task_ref), Location::Task(task2)) => todo!(),
                (Location::Task(task_ref), Location::SinkNode) => todo!(),
            }
        }
    }

    Plan {
        vehicle_tasks: v_plans,
    }
}

impl<'a> HeuristicColgenSolver<'a> {
    pub fn new(problem: &'a Problem) -> Self {
        let _p: hprof::ProfileGuard<'_> = hprof::enter("colgen new");
        let (vehicle_start_nodes, nodes) = txgraph::build_graph(problem, 1.5 * 3600., 30);

        let vehicle_start_nodes = vehicle_start_nodes
            .into_iter()
            .zip(problem.vehicles.iter())
            .map(|(n, v)| (n, v.start_battery))
            .collect::<Vec<_>>();

        let mut time_steps = nodes
            .iter()
            .map(|x| x.state.time)
            .collect::<HashSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        time_steps.sort();
        let time_steps_vehicles = time_steps
            .iter()
            .map(|_| (problem.vehicles.len() as u32))
            .collect();

        let label_buf: Vec<TinyVec<[crate::shortest_path::Label; 20]>> =
            nodes.iter().map(|_| Default::default()).collect();

        HeuristicColgenSolver {
            problem,
            battcycplans: Default::default(),
            columns: Default::default(),
            idxs_buf: Default::default(),
            coeffs_buf: Default::default(),
            nodes,
            time_steps,
            time_steps_vehicles,
            vehicle_start_nodes,
            label_buf,
        }
    }

    fn colgen_fix_one(&mut self) -> Option<usize> {
        let _p = hprof::enter("colgen_fix_one");
        let _p2 = hprof::enter("lp rebuild");
        let mut rmp = LPInstance::new();
        let mut shadow_prices: Vec<f64> = Default::default();

        let veh_cstrs = self
            .time_steps_vehicles
            .iter()
            .map(|n| (*n > 0).then(|| rmp.add_empty_constr(0.0, *n as f64)))
            .collect::<Vec<_>>();
        shadow_prices.extend(veh_cstrs.iter().filter(|x| x.is_some()).map(|_| 0.0));
        let mut time_cost: Vec<f32> = self
            .time_steps_vehicles
            .iter()
            .map(|n| if *n > 0 { 0.0 } else { f32::INFINITY })
            .collect();

        let production_capacity = (0..self.nodes.len())
            .map(|i| {
                production_edge(&self.nodes, i)
                    .filter(|e| !self.nodes[i].outgoing[*e].2.is_infinite()) // remove deactivated edges
                    .map(|e| (e, rmp.add_empty_constr(0.0, 1.0)))
            })
            .collect::<Vec<Option<_>>>();
        shadow_prices.extend(
            production_capacity
                .iter()
                .filter(|x| x.is_some())
                .map(|_| 0.0),
        );

        // Add existing columns
        for (col_idx1, solution) in self.columns.iter().enumerate() {
            create_column(
                &mut self.idxs_buf,
                &mut self.coeffs_buf,
                &self.nodes,
                &self.time_steps,
                &veh_cstrs,
                &production_capacity,
                &solution.path,
            );

            let col_idx2 = rmp.add_column(solution.cost as f64, &self.idxs_buf, &self.coeffs_buf);
            assert!(col_idx1 == col_idx2 as usize);
        }

        struct CurrentBest {
            column: usize,
            value: f64,
            n_iter: usize,
        }

        let mut solution_buf: Vec<f64> = Default::default();
        let mut n_iterations = 0;
        let mut current_best = CurrentBest {
            column: usize::MAX,
            n_iter: 0,
            value: 0.0,
        };

        drop(_p2);

        loop {
            let _val = rmp.optimize(&mut [], &mut []).unwrap();

            // Is it fixing time?
            const STILL_BEST_VALUE_TOLERANCE: f64 = 0.005;
            const STILL_BEST_ITERS: usize = 25;
            const MAX_ITERS: usize = 100;

            let _p2 = hprof::enter("colgen iter after lp");
            while self.columns.len() > solution_buf.len() {
                solution_buf.push(Default::default());
            }
            rmp.get_solution(&mut solution_buf);

            if let Some((best_col, best_value)) = solution_buf
                .iter()
                .enumerate()
                .max_by_key(|(_, x)| OrderedFloat(**x))
            {
                if current_best.column < self.columns.len()
                    && (current_best.column == best_col
                        || current_best.value + STILL_BEST_VALUE_TOLERANCE >= *best_value)
                {
                    current_best.n_iter += 1;

                    if current_best.n_iter >= STILL_BEST_ITERS {
                        // println!("ITERS still best {} after iters {}", STILL_BEST_ITERS, n_iterations);
                        return Some(current_best.column);
                    }
                } else {
                    assert!(current_best.column != best_col);
                    current_best = CurrentBest {
                        column: best_col,
                        value: *best_value,
                        n_iter: 1,
                    };
                }

                if n_iterations >= MAX_ITERS {
                    // println!("ITERS maxed out {}", MAX_ITERS);
                    return Some(best_col);
                }
            }

            n_iterations += 1;
            rmp.get_dual_solution(&mut shadow_prices);

            set_shadow_prices(
                &mut self.nodes,
                &mut time_cost,
                &production_capacity,
                &shadow_prices,
            );

            if let Some(solution) = plan_vehicle(
                u32::MAX,
                &self.nodes,
                &self.vehicle_start_nodes,
                0.0,
                &mut self.label_buf,
                &Default::default(),
                &self.time_steps,
                &time_cost,
            ) {
                const COLGEN_COST_TOL: f32 = 5.0;
                if solution.cost_including_shadow_price >= -COLGEN_COST_TOL {
                    // No more columns to generate.
                    return None;
                }

                create_column(
                    &mut self.idxs_buf,
                    &mut self.coeffs_buf,
                    &self.nodes,
                    &self.time_steps,
                    &veh_cstrs,
                    &production_capacity,
                    &solution.path,
                );

                let col_idx =
                    rmp.add_column(solution.cost as f64, &self.idxs_buf, &self.coeffs_buf);
                assert!(col_idx as usize == self.columns.len());
                self.columns.push(BattCycPlan {
                    cost: solution.cost,
                    path: solution.path,
                });
            } else {
                // Infeasible subproblem. That shouldn't happen (starting and
                // staying at base is always feasible).
                panic!("infeasible");
            }
        }
    }

    pub fn solve_heuristic(mut self) -> Plan {
        let _p = hprof::enter("solve_heuristic");
        while let Some(col_idx) = self.colgen_fix_one() {
            let plan = self.columns.swap_remove(col_idx);
            self.make_fixed_plan(plan);
        }
        Plan {
            vehicle_tasks: vec![],
        }
        // todo!("plan: {:?}", self.battcycplans);
    }

    fn make_fixed_plan(&mut self, plan: BattCycPlan) {
        let _p = hprof::enter("make fixed plan");
        get_plan_edges_in_air(&self.nodes, &plan.path, &self.time_steps, |t_idx| {
            let capacity = &mut self.time_steps_vehicles[t_idx];
            assert!(*capacity > 0);
            *capacity -= 1;
        });

        // TODO put costs in a separate vector so we can iterate nodes while modifying.
        let mut disabled_prod_nodes = Vec::new();
        get_plan_prod_nodes(&self.nodes, &plan.path, |n| {
            let production_edge = production_edge(&self.nodes, n as usize).unwrap();
            disabled_prod_nodes.push((n as usize, production_edge));
        });

        for (n, e) in disabled_prod_nodes {
            assert!(!self.nodes[n].outgoing[e].2.is_infinite());
            self.nodes[n].outgoing[e].2 = f32::INFINITY;
        }

        let _p2 = hprof::enter("remove rest of columns");
        // Check the rest of the columns for overlaps
        self.columns.retain(|c| {
            let mut ok = true;
            get_plan_edges_in_air(&self.nodes, &c.path, &self.time_steps, |t| {
                ok &= self.time_steps_vehicles[t] > 0;
            });
            if !ok {
                return ok;
            }
            get_plan_prod_nodes(&self.nodes, &c.path, |n| {
                let e = production_edge(&self.nodes, n as usize).unwrap();
                ok &= !self.nodes[n as usize].outgoing[e].2.is_infinite();
            });
            ok
        });
    }
}

// Update vehicle shadow prices
fn set_shadow_prices(
    nodes: &mut [Node],
    time_cost: &mut [f32],
    production_capacity: &[Option<(usize, u32)>],
    shadow_prices: &[f64],
) {
    let mut price_idx = 0;
    for x in time_cost.iter_mut().filter(|c| !c.is_infinite()) {
        *x = -(shadow_prices[price_idx] as f32);
        price_idx += 1;
    }

    for (node_idx, constraint) in production_capacity.iter().enumerate() {
        if let Some((edge_idx, _)) = constraint {
            nodes[node_idx].outgoing[*edge_idx].2 = -(shadow_prices[price_idx] as f32);
            price_idx += 1;
        }
    }
}

fn get_plan_edges_in_air(nodes: &[Node], path: &[u32], time: &[i32], mut f: impl FnMut(usize)) {
    // Record vehicle usage
    let mut t_idx = 0;
    for (n1, n2) in path.iter().zip(path.iter().skip(1)) {
        let (s1, s2) = (&nodes[*n1 as usize].state, &nodes[*n2 as usize].state);
        while time[t_idx] < s1.time {
            t_idx += 1;
        }
        let on_ground = (s1.loc == Location::Base && s2.loc == Location::Base)
            || (s1.loc == Location::SinkNode && s2.loc == Location::SinkNode);
        if !on_ground {
            while time[t_idx] < s2.time {
                f(t_idx);
                t_idx += 1;
            }
        }
    }
}

fn get_plan_prod_nodes(nodes: &[Node], path: &[u32], mut f: impl FnMut(u32)) {
    // Record production edges
    for (n1, n2) in path.iter().zip(path.iter().skip(1)) {
        let (s1, s2) = (&nodes[*n1 as usize].state, &nodes[*n2 as usize].state);
        if s1.loc.poi().is_some() && s1.loc == s2.loc {
            f(*n1);
        }
    }
}

fn create_column(
    idxs_buf: &mut Vec<i32>,
    coeffs_buf: &mut Vec<f64>,
    nodes: &[Node],
    ts: &[i32],
    veh_cstrs: &[Option<u32>],
    production_capacity: &[Option<(usize, u32)>],
    path: &[u32],
) {
    // Create new column
    idxs_buf.clear();
    coeffs_buf.clear();

    // Record vehicle usage
    get_plan_edges_in_air(nodes, path, ts, |t_idx| {
        idxs_buf.push(veh_cstrs[t_idx].unwrap() as i32);
        coeffs_buf.push(1.0);
    });

    // Record prodction edges
    get_plan_prod_nodes(nodes, path, |n| {
        let (_, row_idx) = production_capacity[n as usize].as_ref().unwrap();
        idxs_buf.push(*row_idx as i32);
        coeffs_buf.push(1.0);
    });
}

// pub fn solve_old(problem: &Problem) -> Plan {
//     let _p = hprof::enter("colgen solve");
//     let (vehicle_start_nodes, mut nodes) = txgraph::build_graph(problem, 1.5 * 3600., 30);

//     let vehicle_start_nodes = vehicle_start_nodes
//         .into_iter()
//         .zip(problem.vehicles.iter())
//         .map(|(n, v)| (n, v.start_battery))
//         .collect::<Vec<_>>();

//     let mut time_steps = nodes
//         .iter()
//         .map(|x| x.state.time)
//         .collect::<HashSet<_>>()
//         .into_iter()
//         .collect::<Vec<_>>();
//     time_steps.sort();
//     let time_steps = time_steps;

//     let mut rmp = LPInstance::new();
//     let mut shadow_prices: Vec<f64> = Default::default();

//     let veh_cstrs = time_steps
//         .iter()
//         .map(|_t| rmp.add_empty_constr(0.0, problem.vehicles.len() as f64))
//         .collect::<Vec<_>>();
//     shadow_prices.extend(veh_cstrs.iter().map(|_| 0.0));
//     let mut time_cost: Vec<(i32, f32)> = time_steps.iter().map(|t| (*t as i32, 0.0)).collect();

//     let production_capacity = (0..nodes.len())
//         .map(|i| production_edge(&nodes, i).map(|e| (e, rmp.add_empty_constr(0.0, 1.0))))
//         .collect::<Vec<Option<_>>>();
//     shadow_prices.extend(
//         production_capacity
//             .iter()
//             .filter(|x| x.is_some())
//             .map(|_| 0.0),
//     );

//     // let mut last_fix_with_cols = 0;

//     #[allow(unused)]
//     #[derive(Debug)]
//     struct MasterColumn {
//         col_idx: u32,
//         // cost: f32,
//         solution: VehicleSolution,
//     }
//     let mut rmp_columns: Vec<MasterColumn> = Vec::new();
//     // let mut incumbent_values: Vec<f64> = Default::default();
//     let constraints: BTreeSet<Constraint> = BTreeSet::new();
//     let mut label_buf: Vec<TinyVec<[crate::shortest_path::Label; 20]>> =
//         nodes.iter().map(|_| Default::default()).collect();

//     let mut idxs_buf: Vec<i32> = Default::default();
//     let mut coeffs_buf: Vec<f64> = Default::default();

//     // let mut solution_buf: Vec<f64> = Default::default();

//     #[allow(clippy::never_loop)]
//     loop {
//         // Optimize the RMP
//         println!("Solving RMP");
//         // rmp.write_model();
//         // let mut temp = (0..rmp_columns.len()).map(|_| 0.0f64).collect::<Vec<_>>();
//         let objective_value = rmp.optimize(&mut [], &mut shadow_prices).unwrap();
//         println!(" - objective {:.2}", objective_value);
//         // println!("  variable values {:?}", temp);
//         // println!("  shadow prices values {:?}", shadow_prices);

//         // if use_heuristic {
//         //     if rmp_columns.len() >= last_fix_with_cols + 25 {
//         //         while solution_buf.len() < rmp_columns.len() {
//         //             solution_buf.push(Default::default());
//         //         }

//         //         rmp.get_solution(&mut solution_buf);
//         //         assert!(solution_buf.iter().sum::<f64>() >= 1.0 - 1e-3);
//         //         println!("sol {:?}", solution_buf);

//         //         let (selected_col, _) = rmp_columns
//         //             .iter()
//         //             .zip(solution_buf.iter())
//         //             .filter(|(_, x)| **x >= 1e-3)
//         //             .max_by_key(|(c, x)| {
//         //                 OrderedFloat(
//         //                     **x as f32
//         //                         - (nodes[c.solution.path[0] as usize].state.time as f32) / 1800.0,
//         //                 )
//         //             })
//         //             .unwrap();

//         //         println!(
//         //             "wil fix {} t={}",
//         //             selected_col.col_idx,
//         //             (nodes[selected_col.solution.path[0] as usize].state.time as f32)
//         //         );

//         //         panic!();
//         //     }
//         // }

//         let mut n_nonzero_prices = 0;
//         let mut prod_cost_sum = 0.0f32;
//         // Update edge shadow prices
//         for ((n, (e, _row)), p) in production_capacity
//             .iter()
//             .enumerate()
//             .filter_map(|(n, x)| x.map(|e| (n, e)))
//             .zip(shadow_prices.iter().skip(veh_cstrs.len()))
//         {
//             nodes[n].outgoing[e].2 = -(*p as f32);
//             if p.abs() > 1e-3 {
//                 n_nonzero_prices += 1;
//             }
//             prod_cost_sum += *p as f32;
//         }
//         // Update vehicle shadow prices
//         for ((_, x), p) in time_cost.iter_mut().zip(shadow_prices.iter()) {
//             *x = -(*p as f32);
//             if p.abs() > 1e-3 {
//                 n_nonzero_prices += 1;
//             }
//         }
//         println!(" {} nonzero prices", n_nonzero_prices);
//         println!(
//             "   time cost sum {} {:?}",
//             time_cost.iter().map(|(_, c)| *c).sum::<f32>(),
//             &time_cost[..10]
//         );
//         println!("   prod cost sum {}", prod_cost_sum);

//         let mut added_column = false;
//         // Solve the subproblem of finding a new vehicle path
//         let time_cost = time_cost.iter().map(|(_, c)| *c).collect::<Vec<_>>();
//         if let Some(solution) = plan_vehicle(
//             u32::MAX,
//             &nodes,
//             &vehicle_start_nodes,
//             0.0,
//             &mut label_buf,
//             &constraints,
//             &time_steps,
//             &time_cost,
//         ) {
//             const COLGEN_COST_TOL: f32 = 5.0;
//             if solution.cost_including_shadow_price < -COLGEN_COST_TOL {
//                 // Create new column
//                 idxs_buf.clear();
//                 coeffs_buf.clear();

//                 // Record vehicle usage
//                 let mut curr_t = time_steps.iter().enumerate().peekable();
//                 for (s1, s2) in path_to_states(&solution.path, &nodes)
//                     .zip(path_to_states(&solution.path, &nodes).skip(1))
//                 {
//                     while s1.time > curr_t.peek().map(|(_, t)| **t).unwrap() {
//                         curr_t.next();
//                     }
//                     let on_ground = (s1.loc == Location::Base && s2.loc == Location::Base)
//                         || (s1.loc == Location::SinkNode && s2.loc == Location::SinkNode);

//                     if !on_ground {
//                         while s2.time > curr_t.peek().map(|(_, t)| **t).unwrap() {
//                             idxs_buf.push(veh_cstrs[curr_t.peek().unwrap().0] as i32);
//                             coeffs_buf.push(1.0);
//                             curr_t.next();
//                         }
//                     }
//                 }

//                 // for s in plan.states(&nodes) {
//                 //     println!("  {:?}", s);
//                 // }

//                 let n_timeslots = idxs_buf.len();
//                 let t1 = idxs_buf.iter().next().map(|i| time_steps[*i as usize]);
//                 let t2 = idxs_buf.last().map(|i| time_steps[*i as usize]);

//                 // Record production edges
//                 for (n1, n2) in solution.path.iter().zip(solution.path.iter().skip(1)) {
//                     if let Some((e_idx, row_idx)) = &production_capacity[*n1 as usize] {
//                         if nodes[*n1 as usize].outgoing[*e_idx].0 == *n2 {
//                             assert!(nodes[*n1 as usize].state.loc == nodes[*n2 as usize].state.loc);
//                             idxs_buf.push(*row_idx as i32);
//                             coeffs_buf.push(1.0);
//                         }
//                     }
//                 }

//                 let n_production_edges = idxs_buf.len() - n_timeslots;
//                 println!("  adding col c={}/sc={} with {} time slots t={:?} --> t={:?} and {} production edges", solution.cost, solution.cost_including_shadow_price, n_timeslots, t1,t2, n_production_edges);

//                 let col_idx = rmp.add_column(solution.cost as f64, &idxs_buf, &coeffs_buf);
//                 rmp_columns.push(MasterColumn {
//                     col_idx,
//                     // cost: plan.cost,
//                     solution,
//                 });

//                 added_column = true;
//                 // if rmp_columns.len() >= 1000 {
//                 //     panic!("added {} columns", rmp_columns.len())
//                 // }
//             } else {
//                 println!(
//                     "BNP node converged (sc={})",
//                     solution.cost_including_shadow_price
//                 );
//             }
//         } else {
//             println!("Warning : shortest path problem was infeasible");
//         };

//         if !added_column {
//             let _p2 = hprof::enter("price and branch IP");
//             println!("solving price and branch");
//             for c in rmp_columns.iter() {
//                 rmp.set_binary(c.col_idx as i32);
//             }
//             let mut var_values = vec![0.0; rmp_columns.len()];
//             let obj = rmp.optimize(&mut var_values, &mut []).unwrap();
//             println!("solved binary program with obj val {:.2}", obj);
//             assert!(var_values
//                 .iter()
//                 .all(|x| (x.abs() < 1e-4) || (1.0 - x).abs() < 1e-4));
//             let selected_cols = rmp_columns
//                 .iter()
//                 .filter(|x| var_values[x.col_idx as usize] > 0.5)
//                 .collect::<Vec<_>>();
//             for v in selected_cols.iter() {
//                 println!("selected {:?}", v);
//             }

//             drop(_p2);
//             hprof::profiler().print_timing();

//             panic!("finisehd");
//         }
//     }
// }

struct LPInstance {
    ptr: *mut c_void,
}

impl Drop for LPInstance {
    fn drop(&mut self) {
        unsafe {
            highs_sys::Highs_destroy(self.ptr);
        }
    }
}

impl LPInstance {
    pub fn new() -> Self {
        Self {
            ptr: unsafe { highs_sys::Highs_create() },
        }
    }

    pub fn set_binary(&mut self, col_idx: i32) {
        unsafe { highs_sys::Highs_changeColBounds(self.ptr, col_idx, 0.0, 1.0) };
        unsafe {
            highs_sys::Highs_changeColIntegrality(
                self.ptr,
                col_idx,
                highs_sys::kHighsVarTypeInteger,
            )
        };
    }

    pub fn add_column(&mut self, cost: f64, idxs: &[i32], coeffs: &[f64]) -> u32 {
        let new_col_idx = unsafe { highs_sys::Highs_getNumCol(self.ptr) } as u32;
        let inf = unsafe { highs_sys::Highs_getInfinity(self.ptr) };
        assert!(idxs.len() == coeffs.len());
        let retval = unsafe {
            highs_sys::Highs_addCol(
                self.ptr,
                cost,
                0.0,
                inf,
                idxs.len() as i32,
                idxs.as_ptr(),
                coeffs.as_ptr(),
            )
        };
        let status = highs::HighsStatus::try_from(retval);
        assert!(status == Ok(highs::HighsStatus::OK));
        new_col_idx
    }

    pub fn add_empty_constr(&mut self, lb: f64, ub: f64) -> u32 {
        let new_row_idx = unsafe { highs_sys::Highs_getNumRow(self.ptr) } as u32;
        unsafe { highs_sys::Highs_addRow(self.ptr, lb, ub, 0, std::ptr::null(), std::ptr::null()) };
        new_row_idx
    }

    #[allow(unused)]
    pub fn write_model(&mut self) {
        let _p = hprof::enter("write model");
        unsafe {
            highs_sys::Highs_writeModel(
                self.ptr,
                CStr::from_bytes_with_nul("test.lp\0".as_bytes())
                    .unwrap()
                    .as_ptr(),
            )
        };
        println!("model saved");
    }

    pub fn get_solution(&mut self, var_value_out: &mut [f64]) {
        let num_cols = unsafe { highs_sys::Highs_getNumCol(self.ptr) } as usize;

        if var_value_out.len() > 0 {
            assert!(var_value_out.len() == num_cols);

            let null = std::ptr::null_mut();
            unsafe {
                highs_sys::Highs_getSolution(self.ptr, var_value_out.as_mut_ptr(), null, null, null)
            };
        }
    }

    pub fn get_dual_solution(&mut self, row_dual_out: &mut [f64]) {
        let num_rows = unsafe { highs_sys::Highs_getNumRow(self.ptr) } as usize;

        if row_dual_out.len() > 0 {
            println!("Getting row dual");
            assert!(row_dual_out.len() == num_rows);

            let null = std::ptr::null_mut();
            unsafe {
                highs_sys::Highs_getSolution(self.ptr, null, null, null, row_dual_out.as_mut_ptr())
            };
        }
    }

    pub fn optimize(&mut self, var_value_out: &mut [f64], row_dual_out: &mut [f64]) -> Option<f64> {
        let _p = hprof::enter("lp optimize");
        let retval = unsafe { highs_sys::Highs_run(self.ptr) };
        drop(_p);
        let _p = hprof::enter("get_solution");
        let status = highs::HighsStatus::try_from(retval);
        let model_status_retval = unsafe { highs_sys::Highs_getModelStatus(self.ptr) };
        let model_status = highs::HighsModelStatus::try_from(model_status_retval);
        println!("Solved {:?} {:?}", status, model_status);

        // pub const kHighsSolutionStatusNone: HighsInt = 0;
        // pub const kHighsSolutionStatusInfeasible: HighsInt = 1;
        // pub const kHighsSolutionStatusFeasible: HighsInt = 2;
        let mut primal_solution_status: highs_sys::HighsInt = 0;
        let mut dual_solution_status: highs_sys::HighsInt = 0;
        unsafe {
            highs_sys::Highs_getIntInfoValue(
                self.ptr,
                CStr::from_bytes_with_nul("primal_solution_status\0".as_bytes())
                    .unwrap()
                    .as_ptr(),
                &mut primal_solution_status,
            );
            highs_sys::Highs_getIntInfoValue(
                self.ptr,
                CStr::from_bytes_with_nul("dual_solution_status\0".as_bytes())
                    .unwrap()
                    .as_ptr(),
                &mut dual_solution_status,
            );
        };

        let num_cols = unsafe { highs_sys::Highs_getNumCol(self.ptr) } as usize;
        let num_rows = unsafe { highs_sys::Highs_getNumRow(self.ptr) } as usize;

        println!(
            "primal feasible {} dual feasible {}",
            primal_solution_status == 2,
            dual_solution_status == 2
        );

        if var_value_out.len() > 0 {
            assert!(var_value_out.len() == num_cols);

            let null = std::ptr::null_mut();
            unsafe {
                highs_sys::Highs_getSolution(self.ptr, var_value_out.as_mut_ptr(), null, null, null)
            };
        }

        if row_dual_out.len() > 0 {
            println!("Getting row dual");
            assert!(row_dual_out.len() == num_rows);

            let null = std::ptr::null_mut();
            unsafe {
                highs_sys::Highs_getSolution(self.ptr, null, null, null, row_dual_out.as_mut_ptr())
            };
        }

        let mut objective_value = 0.0f64;
        unsafe {
            highs_sys::Highs_getDoubleInfoValue(
                self.ptr,
                CStr::from_bytes_with_nul("objective_function_value\0".as_bytes())
                    .unwrap()
                    .as_ptr(),
                &mut objective_value,
            )
        };

        // println!("getsolution ok");

        Some(objective_value)
    }
}
