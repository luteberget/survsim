use core::f32;
use ordered_float::OrderedFloat;
use std::{
    collections::{HashMap, HashSet, VecDeque},
    time::{Instant, SystemTime, UNIX_EPOCH},
};
use survsim_structs::{
    backend::Task,
    plan::{Cond, Plan, PlanTask},
    problem::Problem,
    report::Location,
    TaskRef,
};
use tinyvec::TinyVec;

use crate::{
    shortest_path::plan_vehicle,
    txgraph::{self, production_edge, Node},
};

// #[test]
// pub fn test() {
//     let _ = env_logger::try_init();
//     let problem =
//         serde_json::from_str(&std::fs::read_to_string("../first_problem.json").unwrap()).unwrap();
//     let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
//     hprof::profiler().print_timing();
//     x.print();
// }

#[test]
pub fn test_flying1() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../regression1.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying2() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../regression2.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying3() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../regression3.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying4() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../regression4.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying5() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../regression5.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying6() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../regression6.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_toocomplicated1() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../no_problem.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_toocomplicated2() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../a_problem.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_toocomplicated3() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../problem_1730299110063.json").unwrap())
            .unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    // x.print();
}

pub fn solve(problem: &Problem) -> Plan {
    std::fs::write(
        format!(
            "problem_{}.json",
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis()
        ),
        serde_json::to_string(&problem).unwrap(),
    )
    .unwrap();

    HeuristicColgenSolver::new(problem).solve_heuristic()
}

#[derive(Debug)]
struct BattCycPlan {
    cost: f32,
    path: Vec<u32>,
}

struct HeuristicColgenSolver<'a> {
    problem: &'a Problem,
    vehicle_start_nodes: Vec<(u32, f32)>,
    fixed_vehicle_starts: Vec<bool>,
    base_node: u32,
    nodes: Vec<Node>,
    time_steps: Vec<i32>,
    time_steps_vehicles: Vec<u32>,
    columns: Vec<BattCycPlan>,
    fixed_plans: Vec<BattCycPlan>,
    label_buf: Vec<TinyVec<[crate::shortest_path::Label; 20]>>,
    idxs_buf: Vec<i32>,
    coeffs_buf: Vec<f64>,
}

fn convert_batt_cyc_plan(problem: &Problem, nodes: &[Node], components: Vec<BattCycPlan>) -> Plan {
    // Sort components by time
    let start_and_end_times = components
        .iter()
        .map(|plan| cyc_start_end(plan, nodes))
        .collect::<Vec<_>>();

    let mut start_time_order = (0..components.len()).collect::<Vec<_>>();
    start_time_order.sort_by_key(|i| start_and_end_times[*i].1 .0);

    let mut vehicles_ready_at_base: tinyvec::TinyVec<[(u32, i32); 10]> = problem
        .vehicles
        .iter()
        .enumerate()
        .filter(|&(_i, v)| (!v.start_airborne))
        .map(|(i, _v)| (i as u32, 0i32))
        .collect();

    let mut v_plans = vec![
        vec![(
            0,
            PlanTask {
                task: Task::Wait,
                finish_cond: Cond::time(f32::INFINITY),
            }
        )];
        problem.vehicles.len()
    ];

    let n_prod_itervals = components
        .iter()
        .map(|x| cyc_production_intervals(x, nodes).len())
        .sum::<usize>();
    println!(
        "COUNT ------- {} batt cycles, {} production intervals",
        components.len(),
        n_prod_itervals
    );

    for plan_idx in start_time_order {
        let plan = &components[plan_idx];
        let ((start_path_idx, end_path_idx), (start_time, end_time)) =
            start_and_end_times[plan_idx];
        // println!("treating plan {:?}", plan);
        let v_init = match nodes[plan.path[0] as usize].state.loc {
            Location::DroneInitial(d) => Some(d),
            _ => None,
        };

        // println!("find ready {:?}", vehicles_ready_at_base);
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

        // println!("new ready {:?}", vehicles_ready_at_base);

        let vplan = &mut v_plans[v_idx];

        assert!(vplan
            .last_mut()
            .unwrap()
            .1
            .finish_cond
            .time
            .unwrap()
            .is_infinite());
        vplan.last_mut().unwrap().1.finish_cond.time = Some(start_time as f32);

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
                (Location::Base, Location::Task(t)) => {
                    vplan.push((
                        s1.state.time,
                        PlanTask {
                            task: Task::Takeoff(t),
                            finish_cond: Cond::external(problem.t_takeoff),
                        },
                    ));
                    vplan.push((
                        s1.state.time,
                        PlanTask {
                            task: Task::GotoPoi(t),
                            finish_cond: Cond::external(
                                ((s2.state.time - s1.state.time) as f32 - problem.t_takeoff)
                                    .max(0.0),
                            ),
                        },
                    ));
                }
                (Location::DroneInitial(_), Location::Task(t)) => {
                    vplan.push((
                        s1.state.time,
                        PlanTask {
                            task: Task::GotoPoi(t),
                            finish_cond: Cond::external((s2.state.time - s1.state.time) as f32),
                        },
                    ));
                }
                (Location::DroneInitial(_), Location::SinkNode)
                | (Location::Task(_), Location::SinkNode) => {
                    vplan.push((
                        s1.state.time,
                        PlanTask {
                            task: Task::ApproachBase,
                            finish_cond: Cond::external(
                                ((s2.state.time - s1.state.time) as f32 - problem.t_takeoff)
                                    .max(0.0),
                            ),
                        },
                    ));
                    vplan.push((
                        s1.state.time,
                        PlanTask {
                            task: Task::Land,
                            finish_cond: Cond::external(problem.t_takeoff),
                        },
                    ));
                }
                (Location::Task(t1), Location::Task(t2)) => {
                    if t1 == t2 {
                        let prev = &mut vplan.last_mut().unwrap().1;
                        if prev.task == Task::WatchPoi(t1) {
                            prev.finish_cond.time = Some(s2.state.time as f32);
                        } else {
                            vplan.push((
                                s1.state.time,
                                PlanTask {
                                    task: Task::WatchPoi(t1),
                                    finish_cond: Cond::time(s2.state.time as f32),
                                },
                            ));
                        }
                    } else {
                        vplan.push((
                            s1.state.time,
                            PlanTask {
                                task: Task::GotoPoi(t2),
                                finish_cond: Cond::external((s2.state.time - s1.state.time) as f32),
                            },
                        ));
                    }
                }
            }
        }

        vplan.push((
            end_time,
            PlanTask {
                task: Task::Wait,
                finish_cond: Cond::time(f32::INFINITY),
            },
        ));
    }

    let mut poi_watchers = problem
        .pois
        .iter()
        .map(|x| (x.task_ref, Vec::new()))
        .collect::<HashMap<TaskRef, Vec<(i32, usize, usize)>>>();

    for (v_idx, vplan) in v_plans.iter().enumerate() {
        for (task_idx, (time, task)) in vplan.iter().enumerate() {
            if let Task::WatchPoi(t) = &task.task {
                poi_watchers
                    .get_mut(t)
                    .unwrap()
                    .push((*time, v_idx, task_idx));
            }
        }
    }

    // for (i, x) in v_plans.iter().enumerate() {
    //     print!("v {}", i);
    //     for x in x {
    //         println!("  - {:?}", x);
    //     }
    // }

    for (_poi, mut watchers) in poi_watchers {
        watchers.sort_by_key(|(t, _, _)| *t);
        for ((_t1, v1, s1), (t2, v2, s2)) in watchers.iter().zip(watchers.iter().skip(1)) {
            let t1_end = &mut v_plans[*v1][*s1].1.finish_cond.time;
            assert!(t1_end.is_some());

            // println!("CHECKING {:?} {:?}  = {}", t1_end, Some(*t2 as f32), t1_end == &Some(*t2 as f32));

            if t1_end == &Some(*t2 as f32) {
                assert!(v1 != v2);
                v_plans[*v1][*s1].1.finish_cond.time = None;
                assert!(v_plans[*v1][*s1].1.finish_cond.task_start.is_none());
                v_plans[*v1][*s1].1.finish_cond.task_start = Some((*v2, *s2));
            }
        }
    }

    Plan {
        vehicle_tasks: v_plans
            .into_iter()
            .map(|x| x.into_iter().map(|(_, t)| t).collect())
            .collect(),
    }
}

fn cyc_start_end(plan: &BattCycPlan, nodes: &[Node]) -> ((usize, usize), (i32, i32)) {
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
    ((start_path_idx, end_path_idx), (start_time, end_time))
}

impl<'a> HeuristicColgenSolver<'a> {
    pub fn new(problem: &'a Problem) -> Self {
        let (base_node, vehicle_start_nodes, nodes) =
            txgraph::build_graph(problem, 1.5 * 3600., 30);

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

        let mut hcs = Self::new_empty(
            problem,
            nodes.clone(),
            base_node,
            vehicle_start_nodes.clone(),
            time_steps.clone(),
        );
        hcs.generate_good_init_columns();
        let mut hcs2 = Self::new_empty(problem, nodes, base_node, vehicle_start_nodes, time_steps);
        hcs2.generate_trivial_init_columns();
        hcs2.columns.extend(hcs.fixed_plans);
        hcs2
    }

    pub fn new_empty(
        problem: &'a Problem,
        nodes: Vec<Node>,
        base_node: u32,
        vehicle_start_nodes: Vec<(u32, f32)>,
        time_steps: Vec<i32>,
    ) -> Self {
        let _p: hprof::ProfileGuard<'_> = hprof::enter("colgen new");

        let time_steps_vehicles = time_steps
            .iter()
            .map(|t| {
                vehicle_start_nodes
                    .iter()
                    .filter(|(n, _)| *t >= nodes[*n as usize].state.time)
                    .count() as u32
            })
            .collect();

        let label_buf: Vec<TinyVec<[crate::shortest_path::Label; 20]>> =
            nodes.iter().map(|_| Default::default()).collect();

        HeuristicColgenSolver {
            problem,
            fixed_plans: Default::default(),
            columns: Default::default(),
            idxs_buf: Default::default(),
            coeffs_buf: Default::default(),
            nodes,
            time_steps,
            time_steps_vehicles,
            fixed_vehicle_starts: vec![false; vehicle_start_nodes.len()],
            vehicle_start_nodes,
            label_buf,
            base_node,
        }
    }

    fn generate_trivial_init_columns(&mut self) {
        for (node, _batt) in self.vehicle_start_nodes.clone().iter() {
            if !matches!(
                self.nodes[*node as usize].state.loc,
                Location::DroneInitial(_)
            ) {
                continue;
            }

            let mut queue: VecDeque<u32> = std::iter::once(*node).collect();
            let mut parent: HashMap<u32, u32> = Default::default();
            const MAX_ITER :usize = 10;
            let mut n_iters = 0;
            while let Some(node) = queue.pop_front() {
                n_iters += 1;
                if n_iters > MAX_ITER {
                    panic!("could not find trivial solution for airborne vehicle");
                }
                if matches!(self.nodes[node as usize].state.loc, Location::SinkNode) {
                    let mut node = node;
                    let mut path = vec![node];
                    while let Some(prev_node) = parent.get(&node) {
                        path.push(*prev_node);
                        node = *prev_node;
                    }
                    path.reverse();
                    let plan = BattCycPlan { cost: 0.0, path };
                    println!("TRIVIAL INIT column {}", cyc_plan_info(&plan, &self.nodes));
                    self.columns.push(plan);
                    break;
                } else {
                    for (next,_,_) in self.nodes[node as usize].outgoing.iter() {
                        if parent.insert(*next, node).is_none() {
                            queue.push_back(*next);
                        }
                    }
                }
            }
        }
    }

    fn generate_good_init_columns(&mut self) {
        // Need to generate at least one column per airborne vehicle to make the LP feasible.
        let time_cost = self.time_steps.iter().map(|_| 0.0).collect::<Vec<f32>>();

        for (v_idx, (node, batt)) in self.vehicle_start_nodes.clone().iter().enumerate() {
            if !matches!(
                self.nodes[*node as usize].state.loc,
                Location::DroneInitial(_)
            ) {
                continue;
            }

            let solution = plan_vehicle(
                u32::MAX,
                &self.nodes,
                &[(*node, *batt)],
                0.0,
                &mut self.label_buf,
                &Default::default(),
                &self.time_steps,
                &time_cost,
            )
            .unwrap();

            let new_plan = BattCycPlan {
                cost: solution.cost,
                path: solution.path,
            };
            println!(
                " Added initial column for vehicle{} {}",
                v_idx,
                cyc_plan_info(&new_plan, &self.nodes)
            );

            self.make_fixed_plan(new_plan);
        }
    }

    fn colgen_fix_one(&mut self) -> Option<usize> {
        let _p = hprof::enter("colgen_fix_one");
        let _p2 = hprof::enter("lp rebuild");
        let mut rmp = crate::lpsolver::LPInstance::new();
        let mut shadow_prices: Vec<f64> = Default::default();

        println!(" Time capacity {:?}", self.time_steps_vehicles);
        println!("FIXED {:?}", self.fixed_vehicle_starts);
        let init_nodes_cstrs = self
            .problem
            .vehicles
            .iter()
            .enumerate()
            .map(|(v_idx, v)| {
                (!self.fixed_vehicle_starts[v_idx] && v.start_airborne).then(|| {
                    (
                        self.vehicle_start_nodes[v_idx].0,
                        rmp.add_empty_constr(1.0, rmp.inf()),
                    )
                })
            })
            .collect::<Vec<Option<(u32, u32)>>>();
        shadow_prices.extend(init_nodes_cstrs.iter().filter(|x| x.is_some()).map(|_| 0.0));

        // println!("init cstrs {:?}", init_nodes_cstrs.iter().filter_map(|x| *x).map(|(x,y)| (self.nodes[x as usize].state,y)).collect::<Vec<_>>());

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

        #[derive(Debug)]
        struct CurrentBest {
            column: usize,
            value: f64,
            n_iter: usize,
        }

        let mut solution_buf: Vec<f64> = Default::default();
        let mut n_iterations = 0;
        let mut n_columns = 0;
        let mut current_best = CurrentBest {
            column: usize::MAX,
            n_iter: 0,
            value: 0.0,
        };

        // let mut col_index: HashMap<Vec<u32>, usize> = Default::default();
        // let mut col_index2: HashMap<String, usize> = Default::default();

        drop(_p2);

        let macro_obj = self.fixed_plans.iter().map(|c| c.cost).sum::<f32>();
        loop {
            // Add columns for plans that haven't been converted into the LP yet.
            while n_columns < self.columns.len() {
                // println!("ADD COL {}", n_columns);
                let solution = &self.columns[n_columns];

                // if col_index.contains_key(&solution.path) {
                //     panic!("Existing solution {}", cyc_plan_info(solution, &self.nodes));
                // } else {
                //     col_index.insert(solution.path.clone(), n_columns);
                // }

                // if let std::collections::hash_map::Entry::Vacant(e) =
                //     col_index2.entry(cyc_plan_info(solution, &self.nodes))
                // {
                //     e.insert(n_columns);
                // } else {
                //     println!(
                //         "Existing SIMILAR solution {}\n{:?}\n{:?}",
                //         cyc_plan_info(solution, &self.nodes),
                //         self.columns[col_index2[&cyc_plan_info(solution, &self.nodes)]]
                //         .path.iter().map(|x| self.nodes[*x as usize].state).collect::<Vec<_>>()
                //         ,
                //         self.columns[n_columns]
                //         .path.iter().map(|x| self.nodes[*x as usize].state).collect::<Vec<_>>()
                //         ,
                //     );
                // }

                // for (col_idx1, solution) in self.columns.iter().enumerate() {
                create_column(
                    &mut self.idxs_buf,
                    &mut self.coeffs_buf,
                    &self.nodes,
                    &self.time_steps,
                    &init_nodes_cstrs,
                    &veh_cstrs,
                    &production_capacity,
                    &solution.path,
                );

                let col_idx2 =
                    rmp.add_column(solution.cost as f64, &self.idxs_buf, &self.coeffs_buf);
                assert!(n_columns == col_idx2 as usize);
                n_columns += 1;
                // }
            }

            rmp.write_model();
            // panic!("ok");

            let micro_obj = rmp.optimize(&mut [], &mut []).unwrap() as f32;
            println!(
                " optimized macro {} ({:.2}) cols {} micro {} ({:.2}) obj={:.2}",
                self.fixed_plans.len(),
                macro_obj,
                self.columns.len(),
                n_iterations,
                micro_obj,
                macro_obj + micro_obj
            );

            // Is it fixing time?
            const STILL_BEST_VALUE_TOLERANCE: f64 = 0.005;
            const STILL_BEST_ITERS: usize = 100;
            const MAX_ITERS: usize = 200;

            let _p2 = hprof::enter("colgen iter after lp");
            while self.columns.len() > solution_buf.len() {
                solution_buf.push(Default::default());
            }
            rmp.get_solution(&mut solution_buf);
            println!("solution {:?}", solution_buf);

            if let Some((best_col, best_value)) = solution_buf
                .iter()
                .enumerate()
                .max_by_key(|(_, x)| OrderedFloat(**x))
            {
                println!("BEST {}<{} {}", best_col, self.columns.len(), best_value);
                if current_best.column < self.columns.len()
                    && (current_best.column == best_col
                        || solution_buf[current_best.column] + STILL_BEST_VALUE_TOLERANCE
                            >= *best_value)
                {
                    current_best.n_iter += 1;

                    if current_best.n_iter >= STILL_BEST_ITERS {
                        println!(
                            "   ITERS still best {} after iters {} value={}",
                            STILL_BEST_ITERS, n_iterations, current_best.value
                        );
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
                    println!("   ITERS maxed out {}  value={}", MAX_ITERS, best_value);
                    return Some(best_col);
                }
            }

            n_iterations += 1;
            rmp.get_dual_solution(&mut shadow_prices);

            set_shadow_prices(
                &mut self.nodes,
                &init_nodes_cstrs,
                &mut time_cost,
                &production_capacity,
                &shadow_prices,
            );

            let start_nodes = self
                .vehicle_start_nodes
                .iter()
                .enumerate()
                .filter(|(i, _x)| !self.fixed_vehicle_starts[*i])
                .map(|(_, x)| *x)
                .chain(std::iter::once((
                    self.base_node,
                    self.problem.battery_capacity,
                )))
                .collect::<Vec<_>>();

            if let Some(solution) = plan_vehicle(
                u32::MAX,
                &self.nodes,
                &start_nodes,
                0.0,
                &mut self.label_buf,
                &Default::default(),
                &self.time_steps,
                &time_cost,
            ) {
                const COLGEN_COST_TOL: f32 = 5.0;
                if solution.cost_including_shadow_price >= -COLGEN_COST_TOL {
                    // No more columns to generate.
                    if current_best.column < self.columns.len()
                        && solution_buf[current_best.column] > STILL_BEST_VALUE_TOLERANCE
                    {
                        return Some(current_best.column);
                    } else {
                        // println!(
                        //     "NO MORE {:?}   {} {} ",
                        //     current_best,
                        //     current_best.column < self.columns.len(),
                        //     solution_buf[current_best.column] > STILL_BEST_VALUE_TOLERANCE
                        // );
                        return None;
                    }
                }

                let new_plan = BattCycPlan {
                    cost: solution.cost,
                    path: solution.path,
                };
                println!(
                    " Added new column {}",
                    cyc_plan_info(&new_plan, &self.nodes)
                );
                self.columns.push(new_plan);
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

        println!(
            " {:?} {:?}",
            self.fixed_vehicle_starts,
            self.problem
                .vehicles
                .iter()
                .map(|v| v.start_airborne)
                .collect::<Vec<_>>()
        );
        assert!(self
            .fixed_vehicle_starts
            .iter()
            .enumerate()
            .all(|(v, f)| *f || !self.problem.vehicles[v].start_airborne));

        convert_batt_cyc_plan(self.problem, &self.nodes, self.fixed_plans)
    }

    fn make_fixed_plan(&mut self, plan: BattCycPlan) {
        println!(" Fixing column {}", cyc_plan_info(&plan, &self.nodes));

        if let Location::DroneInitial(v_idx) = self.nodes[plan.path[0] as usize].state.loc {
            self.fixed_vehicle_starts[v_idx] = true;

            for (_, _, c) in self.nodes[plan.path[0] as usize].outgoing.iter_mut() {
                *c = f32::INFINITY;
            }
        }

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

            if let Location::DroneInitial(v_idx) = self.nodes[c.path[0] as usize].state.loc {
                ok &= !self.fixed_vehicle_starts[v_idx];
            }

            get_plan_edges_in_air(&self.nodes, &c.path, &self.time_steps, |t| {
                ok &= self.time_steps_vehicles[t] > 0;
            });
            get_plan_prod_nodes(&self.nodes, &c.path, |n| {
                let e = production_edge(&self.nodes, n as usize).unwrap();
                ok &= !self.nodes[n as usize].outgoing[e].2.is_infinite();
            });

            if !ok {
                println!("Removing {}", cyc_plan_info(c, &self.nodes));
            }
            ok
        });

        self.fixed_plans.push(plan);
    }
}

fn cyc_plan_info(plan: &BattCycPlan, nodes: &[Node]) -> String {
    let ((_start_path_idx, _end_path_idx), (start_time, end_time)) = cyc_start_end(plan, nodes);
    let prod_intervals = cyc_production_intervals(plan, nodes);

    format!(
        "Plan(nd={:?}, cost={:.2}, time=({},{}), prod={:?})",
        nodes[plan.path[0] as usize].state, plan.cost, start_time, end_time, prod_intervals
    )
}

fn cyc_production_intervals(plan: &BattCycPlan, nodes: &[Node]) -> Vec<(TaskRef, i32, i32)> {
    let mut prod_intervals: Vec<(TaskRef, i32, i32)> = Vec::new();
    for (n1, n2) in plan.path.iter().zip(plan.path.iter().skip(1)) {
        let (s1, s2) = (&nodes[*n1 as usize].state, &nodes[*n2 as usize].state);
        if s1.loc.poi().is_some() && s1.loc == s2.loc {
            if let Some((_t, _t1, t2)) = prod_intervals
                .last_mut()
                .filter(|(t, _, _)| *t == s1.loc.poi().unwrap())
            {
                *t2 = s2.time;
            } else {
                prod_intervals.push((s1.loc.poi().unwrap(), s1.time, s2.time));
            }
        }
    }
    prod_intervals
}

// Update vehicle shadow prices
fn set_shadow_prices(
    nodes: &mut [Node],
    init_cstrs: &[Option<(u32, u32)>],
    time_cost: &mut [f32],
    production_capacity: &[Option<(usize, u32)>],
    shadow_prices: &[f64],
) {
    let mut price_idx = 0;

    let mut total_init_cost = 0.0;
    for (node_idx, _row_idx) in init_cstrs.iter().filter_map(|x| x.as_ref()) {
        total_init_cost += -(shadow_prices[price_idx] as f32);
        if shadow_prices[price_idx].abs() > 1e-3 {
            println!(
                "setting shadow price of {:?} to {}",
                nodes[*node_idx as usize].state,
                -(shadow_prices[price_idx] as f32)
            );
        }

        for (_, _, x) in nodes[*node_idx as usize].outgoing.iter_mut() {
            *x = -(shadow_prices[price_idx] as f32);
        }
        price_idx += 1;
    }

    let mut total_vehicle_cost = 0.0;
    for x in time_cost.iter_mut().filter(|c| !c.is_infinite()) {
        *x = -(shadow_prices[price_idx] as f32);
        price_idx += 1;
        total_vehicle_cost += *x;
    }

    let mut total_prod_cost = 0.0;
    for (node_idx, constraint) in production_capacity.iter().enumerate() {
        if let Some((edge_idx, _)) = constraint {
            nodes[node_idx].outgoing[*edge_idx].2 = -(shadow_prices[price_idx] as f32);
            total_prod_cost += -(shadow_prices[price_idx] as f32);
            price_idx += 1;
        }
    }

    println!(
        "  total initcost {} vcost {} pcost {}",
        total_init_cost, total_vehicle_cost, total_prod_cost
    );
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

#[allow(clippy::too_many_arguments)]
fn create_column(
    idxs_buf: &mut Vec<i32>,
    coeffs_buf: &mut Vec<f64>,
    nodes: &[Node],
    ts: &[i32],
    init_cstrs: &[Option<(u32, u32)>],
    veh_cstrs: &[Option<u32>],
    production_capacity: &[Option<(usize, u32)>],
    path: &[u32],
) {
    // Create new column
    idxs_buf.clear();
    coeffs_buf.clear();

    // Check if we are in an initial node.
    let first_loc = nodes[path[0] as usize].state.loc;
    if matches!(first_loc, Location::DroneInitial(_)) {
        println!("initial node {} {:?}  {:?}", path[0], first_loc, init_cstrs);
        let init_cstr = init_cstrs
            .iter()
            .position(|x| x.filter(|(node, _)| *node == path[0]).is_some())
            .unwrap();

        idxs_buf.push(init_cstrs[init_cstr].unwrap().1 as i32);
        coeffs_buf.push(1.0);
    }

    // Record vehicle usage
    get_plan_edges_in_air(nodes, path, ts, |t_idx| {
        idxs_buf.push(veh_cstrs[t_idx].unwrap() as i32);
        coeffs_buf.push(1.0);
    });

    // Record production edges
    get_plan_prod_nodes(nodes, path, |n| {
        let (_, row_idx) = production_capacity[n as usize].as_ref().unwrap();
        idxs_buf.push(*row_idx as i32);
        coeffs_buf.push(1.0);
    });
}
