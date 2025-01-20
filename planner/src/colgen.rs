#![cfg(feature = "highs")]

use core::{f32, f64};
use ordered_float::OrderedFloat;
use std::{
    collections::{HashMap, HashSet, VecDeque},
    time::{SystemTime, UNIX_EPOCH},
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
    decomposition::{
        convert_batt_cyc_plan, cyc_plan_info, get_plan_edges_in_air, get_plan_prod_nodes,
        BattCycPlan,
    },
    extsolvers::highs::HighsSolverInstance,
    greedy,
    shortest_path::plan_vehicle,
    txgraph::{self, production_edge, Node, DEFAULT_TIME_HORIZON},
};

pub fn solve(problem: &Problem) -> ((f32, f32), Plan) {
    // std::fs::write(
    //     format!(
    //         "problem_{}.json",
    //         SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis()
    //     ),
    //     serde_json::to_string(&problem).unwrap(),
    // )
    // .unwrap();

    HeuristicColgenSolver::new(problem).solve_heuristic()
}

pub struct HeuristicColgenSolver<'a> {
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

impl<'a> HeuristicColgenSolver<'a> {
    pub fn new(problem: &'a Problem) -> Self {
        let (base_node, vehicle_start_nodes, nodes) =
            txgraph::build_graph(problem, DEFAULT_TIME_HORIZON, 30, true);

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

    pub fn add_greedy_columns(mut self) -> Self {
        let (_total_cost, cyc_plans) = greedy::get_greedy_cycles(
            self.problem,
            self.base_node,
            &self.vehicle_start_nodes,
            &self.nodes,
        );
        self.columns.extend(cyc_plans);
        self
    }

    pub fn new_empty(
        problem: &'a Problem,
        nodes: Vec<Node>,
        base_node: u32,
        vehicle_start_nodes: Vec<(u32, f32)>,
        time_steps: Vec<i32>,
    ) -> Self {
        #[cfg(feature = "prof")]
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
            const MAX_ITER: usize = 10;
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
                    let plan = BattCycPlan { cost: 1000.0, path };
                    println!("TRIVIAL INIT column {}", cyc_plan_info(&plan, &self.nodes));
                    self.columns.push(plan);
                    break;
                } else {
                    for (next, _, _) in self.nodes[node as usize].outgoing.iter() {
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

            // self.make_fixed_plan(new_plan);
            self.fixed_plans.push(new_plan);
        }
    }

    fn solve_binary_program(&mut self, timeout: f64) -> ((f32, f32), Plan) {
        let mut rmp = HighsSolverInstance::new();
        let mut solution_buf: Vec<f64> = Default::default();

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

        // println!("init cstrs {:?}", init_nodes_cstrs.iter().filter_map(|x| *x).map(|(x,y)| (self.nodes[x as usize].state,y)).collect::<Vec<_>>());

        let veh_cstrs = self
            .time_steps_vehicles
            .iter()
            .map(|n| (*n > 0).then(|| rmp.add_empty_constr(0.0, *n as f64)))
            .collect::<Vec<_>>();

        let production_capacity = (0..self.nodes.len())
            .map(|i| {
                production_edge(&self.nodes, i)
                    .filter(|e| !self.nodes[i].outgoing[*e].2.is_infinite()) // remove deactivated edges
                    .map(|e| (e, rmp.add_empty_constr(0.0, 1.0)))
            })
            .collect::<Vec<Option<_>>>();

        for batt_cyc in self.columns.iter() {
            create_column(
                &mut self.idxs_buf,
                &mut self.coeffs_buf,
                &self.nodes,
                &self.time_steps,
                &init_nodes_cstrs,
                &veh_cstrs,
                &production_capacity,
                &batt_cyc.path,
            );
            let var = rmp.add_column(batt_cyc.cost as f64, &self.idxs_buf, &self.coeffs_buf);
            rmp.set_binary(var);
        }

        rmp.set_time_limit(timeout);
        let obj = rmp.optimize(&mut solution_buf, &mut []).unwrap().0 as f32;
        let cycles = self
            .columns
            .iter()
            .zip(solution_buf.iter())
            .filter(|(p, v)| (**v > 0.5))
            .map(|(p, _)| p.clone())
            .collect::<Vec<_>>();

        let (plan, _) =
            convert_batt_cyc_plan(self.problem, &self.nodes, &self.vehicle_start_nodes, cycles);

        ((obj, f32::NEG_INFINITY), plan)
    }

    fn generate_columns_from_pricing(
        &mut self,
        stop_with_good_column: bool,
        timeout: f64,
    ) -> Option<usize> {
        #[cfg(feature = "prof")]
        let _p = hprof::enter("colgen_fix_one");
        #[cfg(feature = "prof")]
        let _p2 = hprof::enter("lp rebuild");
        let mut rmp = HighsSolverInstance::new();
        let mut shadow_prices: Vec<f64> = Default::default();
        let start_time = std::time::Instant::now();

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

        let mut col_index: HashMap<Vec<u32>, usize> = Default::default();
        let mut col_index2: HashMap<String, usize> = Default::default();

        #[cfg(feature = "prof")]
        drop(_p2);

        let macro_obj = self.fixed_plans.iter().map(|c| c.cost).sum::<f32>();
        loop {
            // Add columns for plans that haven't been converted into the LP yet.
            while n_columns < self.columns.len() {
                // println!("ADD COL {}", n_columns);
                let solution = &self.columns[n_columns];

                if !cyc_plan_info(solution, &self.nodes).contains("DroneInitial") {
                    if col_index.contains_key(&solution.path) {
                        panic!("Existing solution {}", cyc_plan_info(solution, &self.nodes));
                    } else {
                        col_index.insert(solution.path.clone(), n_columns);
                    }

                    if let std::collections::hash_map::Entry::Vacant(e) =
                        col_index2.entry(cyc_plan_info(solution, &self.nodes))
                    {
                        e.insert(n_columns);
                    } else {
                        panic!(
                            "Existing SIMILAR solution {}\n{:?}\n{:?}",
                            cyc_plan_info(solution, &self.nodes),
                            self.columns[col_index2[&cyc_plan_info(solution, &self.nodes)]]
                                .path
                                .iter()
                                .map(|x| self.nodes[*x as usize].state)
                                .collect::<Vec<_>>(),
                            self.columns[n_columns]
                                .path
                                .iter()
                                .map(|x| self.nodes[*x as usize].state)
                                .collect::<Vec<_>>(),
                        );
                    }
                }
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

            let micro_obj = rmp.optimize(&mut [], &mut []).unwrap().0 as f32;
            println!(
                " optimized macro {} ({:.2}) cols {} micro {} ({:.2}) obj={:.2}",
                self.fixed_plans.len(),
                macro_obj,
                self.columns.len(),
                n_iterations,
                micro_obj,
                macro_obj + micro_obj
            );

            let _p2 = hprof::enter("colgen iter after lp");
            #[cfg(feature = "prof")]
            while self.columns.len() > solution_buf.len() {
                solution_buf.push(Default::default());
            }
            rmp.get_solution(&mut solution_buf);
            println!("solution {:?}", &solution_buf[..solution_buf.len().min(20)]);
            let col_ranking_cost = solution_buf
                .iter()
                .enumerate()
                .map(|(i, x)| {
                    OrderedFloat(if *x < 0.01 {
                        f64::INFINITY
                    } else {
                        self.columns[i].cost as f64 * x
                    })
                })
                .collect::<Vec<_>>();
            println!("COL RANKING {:?}", col_ranking_cost);

            let timing_out = start_time.elapsed().as_secs_f64() >= timeout;

            // Is it fixing time?
            const STILL_BEST_VALUE_TOLERANCE: f64 = 0.005;
            if stop_with_good_column {
                // const STILL_BEST_ITERS: usize = 500;
                const STILL_BEST_ITERS: usize = 1;
                const MAX_ITERS: usize = 1;

                if let Some((best_col, best_value)) = col_ranking_cost
                    .iter()
                    .enumerate()
                    .min_by_key(|(_i, x)| **x)
                {
                    println!(
                        "BEST {}<{} {}   (costs {:?})",
                        best_col,
                        self.columns.len(),
                        best_value,
                        self.columns.iter().map(|c| c.cost).collect::<Vec<_>>()
                    );
                    if current_best.column < self.columns.len()
                        && (current_best.column == best_col
                            || col_ranking_cost[current_best.column]
                                * (1.0 - STILL_BEST_VALUE_TOLERANCE)
                                <= *best_value)
                    {
                        current_best.n_iter += 1;

                        let iter_limit = if self.fixed_plans.is_empty() {
                            MAX_ITERS
                        } else {
                            STILL_BEST_ITERS
                        };
                        if current_best.n_iter >= iter_limit {
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
                            value: best_value.0,
                            n_iter: 1,
                        };
                    }

                    if timing_out || n_iterations >= MAX_ITERS {
                        println!("   ITERS maxed out {}  value={}", MAX_ITERS, best_value);
                        return Some(best_col);
                    }
                }
            }

            if timing_out {
                return None;
            }

            println!(" BEST COL {:?}", current_best);

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

    pub fn solve_price_and_branch(mut self, timeout: f64) -> ((f32, f32), Plan) {
        #[cfg(feature = "prof")]
        let _p = hprof::enter("solve_price_and_branch");
        let start_time = std::time::Instant::now();
        const GEN_RATIO: f64 = 0.8;
        self.generate_columns_from_pricing(false, timeout * GEN_RATIO);
        self.solve_binary_program(timeout - start_time.elapsed().as_secs_f64())
    }

    pub fn solve_heuristic(mut self) -> ((f32, f32), Plan) {
        #[cfg(feature = "prof")]
        let _p = hprof::enter("solve_heuristic");
        while let Some(col_idx) = self.generate_columns_from_pricing(true, f64::INFINITY) {
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

        let total_cost = self.fixed_plans.iter().map(|x| x.cost).sum::<f32>();
        let (plan, _) = convert_batt_cyc_plan(
            self.problem,
            &self.nodes,
            &self.vehicle_start_nodes,
            self.fixed_plans,
        );
        ((total_cost, f32::NEG_INFINITY), plan)
    }

    fn make_fixed_plan(&mut self, plan: BattCycPlan) {
        println!(" Fixing column {}", cyc_plan_info(&plan, &self.nodes));

        if let Location::DroneInitial(v_idx) = self.nodes[plan.path[0] as usize].state.loc {
            self.fixed_vehicle_starts[v_idx] = true;

            for (_, _, c) in self.nodes[plan.path[0] as usize].outgoing.iter_mut() {
                *c = f32::INFINITY;
            }
        }

        #[cfg(feature = "prof")]
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

        #[cfg(feature = "prof")]
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
