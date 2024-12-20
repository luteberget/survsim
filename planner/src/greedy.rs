
use std::collections::HashSet;
use crate::shortest_path::plan_vehicle;
use survsim_structs::report::Location;
use survsim_structs::{plan::Plan, problem::Problem};
use tinyvec::TinyVec;

use crate::txgraph::{self, production_edge};
use crate::decomposition::{convert_batt_cyc_plan, cyc_plan_info, get_plan_edges_in_air, get_plan_prod_nodes, BattCycPlan};


pub fn solve_greedy_cycles(problem: &Problem) -> Plan {

    let (base_node, vehicle_start_nodes, mut nodes) = txgraph::build_graph(problem, 1.5 * 3600., 30);

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

    let mut time_steps_vehicles = time_steps
        .iter()
        .map(|t| {
            vehicle_start_nodes
                .iter()
                .filter(|(n, _)| *t >= nodes[*n as usize].state.time)
                .count() as u32
        })
        .collect::<Vec<_>>();

    let mut label_buf: Vec<TinyVec<[crate::shortest_path::Label; 20]>> =
        nodes.iter().map(|_| Default::default()).collect();
    let mut airborne_vehicles = problem.vehicles.iter().map(|x| x.start_airborne).collect::<Vec<_>>();
    let mut cyc_plans: Vec<BattCycPlan> = Vec::new();

    let mut time_cost: Vec<f32> = time_steps_vehicles
        .iter()
        .map(|n| if *n > 0 { 0.0 } else { f32::INFINITY })
        .collect();

    let mut airborne_only = true;

    'add: loop {
        let mut start_nodes = vehicle_start_nodes
            .iter()
            .enumerate()
            .filter(|(i, _x)| airborne_vehicles[*i])
            .map(|(_, x)| *x)
            .collect::<Vec<_>>();
        let finished_airborne = start_nodes.is_empty();

        if finished_airborne && airborne_only {
            airborne_only = false;
        }

        if !airborne_only {
            start_nodes.push((base_node, problem.battery_capacity));
        }

        let start_nodes = start_nodes;

        if let Some(solution) = plan_vehicle(
            u32::MAX,
            &nodes,
            &start_nodes,
            0.0,
            &mut label_buf,
            &Default::default(),
            &time_steps,
            &time_cost,
        ) {
            let plan = BattCycPlan { cost: solution.cost, path: solution.path };

            println!("finished air {} plancost {}", finished_airborne, plan.cost);

            if !airborne_only && plan.cost >= -5.0 {
                if finished_airborne {
                    break 'add;
                } else {
                    airborne_only = true;
                    continue 'add;
                }
            }

            println!(" Fixing battcyc {}", cyc_plan_info(&plan, &nodes));

            if let Location::DroneInitial(v_idx) = nodes[plan.path[0] as usize].state.loc {
                airborne_vehicles[v_idx] = false;

                for (_, _, c) in nodes[plan.path[0] as usize].outgoing.iter_mut() {
                    *c = f32::INFINITY;
                }
            }

            #[cfg(feature="prof")]
            let _p = hprof::enter("make fixed plan");
            get_plan_edges_in_air(&nodes, &plan.path, &time_steps, |t_idx| {
                let capacity = &mut time_steps_vehicles[t_idx];
                assert!(*capacity > 0);
                *capacity -= 1;
                if *capacity == 0 {
                    time_cost[t_idx] = f32::INFINITY;
                }
            });

            // TODO put costs in a separate vector so we can iterate nodes while modifying.
            let mut disabled_prod_nodes = Vec::new();
            get_plan_prod_nodes(&nodes, &plan.path, |n| {
                let production_edge = production_edge(&nodes, n as usize).unwrap();
                disabled_prod_nodes.push((n as usize, production_edge));
            });

            for (n, e) in disabled_prod_nodes {
                assert!(!nodes[n].outgoing[e].2.is_infinite());
                nodes[n].outgoing[e].2 = f32::INFINITY;
            }

            cyc_plans.push(plan);
        } else {
            panic!("infeasible");
        }
    }

    convert_batt_cyc_plan(problem, &nodes, cyc_plans)
}
