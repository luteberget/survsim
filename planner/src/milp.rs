#![cfg(feature = "highs")]
use core::f32;
use std::collections::{HashMap, HashSet};

use crate::{
    extsolvers::LPSolver, txgraph::{self, production_edge, DEFAULT_TIME_HORIZON}
};
use survsim_structs::{plan::Plan, problem::Problem, report::Location};



pub fn solve<LP :LPSolver>(problem: &Problem, timeout :f64) -> ((f32, f32), Plan) {
    let (_base_node, vehicle_start_nodes, nodes) =
        txgraph::build_graph(problem, DEFAULT_TIME_HORIZON, 30, false);

    let ts = {
        let mut ts = nodes
            .iter()
            .map(|n| n.state.time)
            .collect::<HashSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        ts.sort();
        ts
    };
    let last_time = *ts.last().unwrap();

    let mut lp = LP::new();

    #[derive(Debug)]
    enum VarInfo {
        Edge(usize, usize, usize),
        Battery(usize, i32),
    }

    let mut var_info: Vec<VarInfo> = Default::default();
    let mut edge_vars: Vec<Vec<Vec<_>>> = vec![vec![vec![]; nodes.len()]; problem.vehicles.len()];
    let mut incoming: Vec<Vec<Vec<(usize, LP::Var)>>> =
        vec![vec![vec![]; nodes.len()]; problem.vehicles.len()];
    let mut outgoing: Vec<Vec<Vec<(usize, LP::Var)>>> =
        vec![vec![vec![]; nodes.len()]; problem.vehicles.len()];

    for (v_idx, vehicle) in problem.vehicles.iter().enumerate() {
        // Edge variables
        for (source_state, node) in nodes.iter().enumerate() {
            for (edge_idx, (target_state, edge_data, added_cost)) in
                node.outgoing.iter().enumerate()
            {
                assert!(*added_cost == 0.0);
                let var = lp.add_var(edge_data.cost as f64);
                var_info.push(VarInfo::Edge(v_idx, source_state, edge_idx));
                lp.set_binary(var);
                edge_vars[v_idx][source_state].push(var);
                incoming[v_idx][*target_state as usize].push((source_state, var));
                outgoing[v_idx][source_state].push((*target_state as usize, var));
            }
        }

        // Vehicle flow constraints
        for (node_idx, node) in nodes.iter().enumerate() {
            let net_flow = if node_idx == vehicle_start_nodes[v_idx] as usize {
                1.0
            } else if node.state.loc == Location::Base && node.state.time == last_time {
                -1.0
            } else {
                0.0
            };

            let mut idxs = Vec::new();
            let mut coeffs = Vec::new();
            for (_, var) in incoming[v_idx][node_idx].iter() {
                idxs.push(*var);
                coeffs.push(1.0);
            }

            for (_, var) in outgoing[v_idx][node_idx].iter() {
                idxs.push(*var);
                coeffs.push(-1.0);
            }

            lp.add_constraint(-net_flow, -net_flow, &idxs, &coeffs);
        }

        // Battery constraint
        let start_time = nodes[vehicle_start_nodes[v_idx] as usize].state.time;
        let mut battery_level_vars: HashMap<i32, _> = Default::default();
        battery_level_vars.insert(start_time, {
            let var = lp.add_var(0.0);
            var_info.push(VarInfo::Battery(v_idx, start_time));
            lp.set_bounds(
                var,
                vehicle.start_battery as f64,
                vehicle.start_battery as f64,
            );
            var
        });

        for (t_idx, next_t) in ts.iter().enumerate().skip_while(|(_, t)| **t <= start_time) {
            // Starting from the first time *after* the start_time.
            let prev_t = ts[t_idx - 1];
            let prev_var = battery_level_vars[&prev_t];
            let next_var = *battery_level_vars.entry(*next_t).or_insert_with(|| {
                let var = lp.add_var(0.0);
                var_info.push(VarInfo::Battery(v_idx, *next_t));
                lp.set_bounds(var, 0.0, problem.battery_capacity as f64);
                var
            });

            let mut idxs = Vec::new();
            let mut coeffs = Vec::new();
            idxs.push(prev_var);
            coeffs.push(1.0);
            idxs.push(next_var);
            coeffs.push(-1.0);

            // add contribution or negative from edges
            for (source_idx, source_node) in nodes.iter().enumerate() {
                if source_node.state.time > prev_t {
                    continue;
                }

                for (edge_idx, (target_idx, data, _)) in source_node.outgoing.iter().enumerate() {
                    let target_node = &nodes[*target_idx as usize];

                    if target_node.state.time < *next_t {
                        continue;
                    }

                    let is_base = matches!(source_node.state.loc, Location::Base)
                        && matches!(target_node.state.loc, Location::Base);
                    assert!(
                        !is_base
                            || (source_node.state.time == prev_t
                                && target_node.state.time == *next_t)
                    );

                    let edge_var = edge_vars[v_idx][source_idx][edge_idx];

                    if is_base {
                        // Recharge (add battery_capacity incoming)
                        idxs.push(edge_var);
                        coeffs.push(problem.battery_capacity as f64);
                    } else {
                        // Discharge battery on the first time step (reduce)
                        if source_node.state.time == prev_t {
                            idxs.push(edge_var);
                            coeffs.push(-data.batt as f64);
                        }
                        // If it's not the first time step, we have no contribution from the edge,
                        // resulting in `b_{t+1} <= b_t`.
                    }
                }
            }

            lp.add_constraint(0.0, lp.inf(), &idxs, &coeffs);
        }
    }

    // Capacity constraints
    for (node_idx, _node) in nodes.iter().enumerate() {
        if let Some(edge_idx) = production_edge(&nodes, node_idx) {
            let vars = edge_vars
                .iter()
                .map(|edges| edges[node_idx][edge_idx])
                .collect::<Vec<_>>();
            lp.add_constraint(-lp.inf(), 1.0, &vars, &vec![1.0; vars.len()]);
        }
    }

    let mut sol = vec![0.0; lp.num_vars()];
    println!("writing model");
    lp.write_model();
    println!("optimizing");
    lp.set_time_limit(timeout);
    let result = lp.optimize();
    // println!("solution {:?}", sol);

    // for (var, val) in sol.iter().enumerate() {
    //     if *val > -1e-9 {
    //         match var_info[var] {
    //             VarInfo::Edge(v_idx, node_idx, edge_idx) => {
    //                 println!(
    //                     " x{} v{}  {:?}--{:?}  - {}",
    //                     var + 1,
    //                     v_idx,
    //                     nodes[node_idx].state,
    //                     nodes[nodes[node_idx].outgoing[edge_idx].0 as usize].state,
    //                     *val
    //                 );
    //             }
    //             VarInfo::Battery(_, _) => {},
    //         }
    //     }
    // }

    (
        result.map(|(x,y,_)| (x as f32, y as f32)).unwrap_or((f32::INFINITY, f32::NEG_INFINITY)),
        Plan {
            vehicle_tasks: vec![],
        },
    )
}
