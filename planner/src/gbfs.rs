#![deny(clippy::print_stdout, clippy::print_stderr)]
#![cfg(feature="gbfs")]

use std::collections::BTreeSet;

use log::{debug, info, trace};
use serde_json::json;
use survsim_structs::{problem::Problem, TaskRef};
use tinyvec::TinyVec;

use crate::{
    txgraph::{build_graph, Constraint, Node},
    VehicleSolution,
};

struct World<'a> {
    problem: &'a Problem,
    vehicle_start_nodes: Vec<u32>,
    nodes: Vec<Node>,
    solutions: Vec<VehicleSolution>,
    constraints: BTreeSet<Constraint>,
    label_buf: Vec<TinyVec<[crate::shortest_path::Label; 20]>>,
}

impl<'a> World<'a> {
    pub fn new(problem: &'a Problem, time_horizon: f32, time_scale: i32) -> Self {
        let (_base_node, vehicle_start_nodes, nodes) = build_graph(problem, time_horizon, time_scale);

        let constraints = Default::default();
        let mut label_buf: Vec<TinyVec<[crate::shortest_path::Label; 20]>> =
            nodes.iter().map(|_| Default::default()).collect();

        let initial_solutions = vehicle_start_nodes
            .iter()
            .enumerate()
            .map(|(v, s)| {
                crate::shortest_path::plan_vehicle(
                    v as u32,
                    &nodes,
                    &[(*s, problem.vehicles[v].start_battery)],
                    problem.battery_capacity,
                    &mut label_buf,
                    &constraints,
                    &[],&[],
                ).unwrap()
            })
            .collect::<Vec<_>>();

        Self {
            nodes,
            problem,
            solutions: initial_solutions,
            vehicle_start_nodes,
            // time_scale,
            label_buf,
            constraints,
        }
    }

    pub fn permanent_constraints(&mut self, cs: impl Iterator<Item = Constraint>) {
        let mut dirty_vehicles: tinyvec::TinyVec<[u32; 10]> = Default::default();

        for c in cs {
            assert!(self.constraints.insert(c));
            if !dirty_vehicles.contains(&c.vehicle) {
                dirty_vehicles.push(c.vehicle);
            }
        }

        for v in dirty_vehicles {
            self.solutions[v as usize] = crate::shortest_path::plan_vehicle(
                v,
                &self.nodes,
                &[(
                    self.vehicle_start_nodes[v as usize],
                    self.problem.vehicles[v as usize].start_battery,
                )],
                self.problem.battery_capacity,
                &mut self.label_buf,
                &self.constraints,
                &[],&[]
            ).unwrap();
        }
    }

    pub fn temporary_constraints(&mut self, cs: &[Constraint]) -> f32 {
        let mut dirty_vehicles: tinyvec::TinyVec<[u32; 10]> = Default::default();
        for c in cs.iter() {
            assert!(self.constraints.insert(*c));

            // TODO: would only be dirty if the incumbent solution uses the edge.
            if !dirty_vehicles.contains(&c.vehicle) {
                dirty_vehicles.push(c.vehicle);
            }
        }

        let cost = dirty_vehicles
            .into_iter()
            .map(|v| {
                let v_cost = crate::shortest_path::plan_vehicle(
                    v,
                    &self.nodes,
                    &[(
                        self.vehicle_start_nodes[v as usize],
                        self.problem.vehicles[v as usize].start_battery,
                    )],
                    self.problem.battery_capacity,
                    &mut self.label_buf,
                    &self.constraints,
                    &[],&[]
                ).unwrap()
                .cost;

                v_cost - self.solutions[v as usize].cost
            })
            .sum::<f32>();

        for c in cs.iter() {
            assert!(self.constraints.remove(c));
        }
        cost
    }
}

struct Branch {
    constraints: Vec<Constraint>,
    lb: f32,
}

#[derive(Debug)]
struct Conflict {
    vehicles: tinyvec::TinyVec<[(u32, i32); 10]>,
    time: (i32, i32),
    poi: TaskRef,
}

fn find_conflict(world: &World) -> Option<Conflict> {
    let _p = hprof::enter("find_conflict");
    let mut conflicts: Vec<Conflict> = Default::default();

    #[derive(Default, Debug)]
    struct PoiState {
        task: TaskRef,
        vehicles_since: TinyVec<[(u32, i32); 10]>,
    }

    let mut pois: TinyVec<[PoiState; 10]> = world
        .problem
        .pois
        .iter()
        .map(|_| Default::default())
        .collect();

    let mut vehicle_poi: TinyVec<[Option<TaskRef>; 4]> = world
        .problem
        .vehicles
        .iter()
        .map(|_| Default::default())
        .collect();

    let mut vehicle_path_idx: TinyVec<[(u32, u32); 10]> = world
        .problem
        .vehicles
        .iter()
        .enumerate()
        .map(|(v, _)| (v as u32, 0))
        .collect();
    vehicle_path_idx.sort_by_key(|(v, i)| {
        world.nodes[world.solutions[*v as usize].path[*i as usize] as usize]
            .state
            .time
    });

    let final_time = world.nodes.last().unwrap().state.time;
    'find_conflicts: loop {
        if vehicle_path_idx.is_empty() {
            break 'find_conflicts;
        }
        let (v_idx, path_idx) = &mut vehicle_path_idx[0];
        assert!((*path_idx as usize) + 1 < world.solutions[*v_idx as usize].path.len());

        let node = world.solutions[*v_idx as usize].path[*path_idx as usize];
        let next_node = world.solutions[*v_idx as usize].path[*path_idx as usize + 1];
        trace!(
            "find_conflicts: updating {} at path idx {} {:?} --  {:?}",
            v_idx,
            path_idx,
            world.nodes[node as usize].state,
            world.nodes[next_node as usize].state
        );
        // let (_, edge_data, _) = world.nodes[node as usize]
        //     .outgoing
        //     .iter()
        //     .find(|(x, e, b)| *x == next_node)
        //     .unwrap();

        let curr_poi = world.nodes[node as usize].state.loc.poi().filter(|_| {
            world.nodes[node as usize].state.loc.poi()
                == world.nodes[next_node as usize].state.loc.poi()
        });

        if curr_poi == vehicle_poi[*v_idx as usize] {
            // Same as last.
        } else {
            assert!(curr_poi.is_none() ^ vehicle_poi[*v_idx as usize].is_none());
            let curr_time = world.nodes[node as usize].state.time;
            // Exit
            if let Some(exit_poi) = vehicle_poi[*v_idx as usize] {
                let poi: &mut PoiState = pois.iter_mut().find(|p| p.task == exit_poi).unwrap();
                let nonzero_time_vehicles: TinyVec<[(u32, i32); 10]> = poi
                    .vehicles_since
                    .iter()
                    .filter(|(_v, t)| *t < curr_time)
                    .copied()
                    .collect();

                if nonzero_time_vehicles.len() > 1 {
                    // Conflict
                    let start_time = nonzero_time_vehicles.iter().map(|(_, t)| *t).max().unwrap();
                    conflicts.push(Conflict {
                        poi: exit_poi,
                        vehicles: nonzero_time_vehicles,
                        time: (start_time, curr_time),
                    });

                    if conflicts.len() >= 10
                        || start_time - conflicts.iter().map(|c| c.time.0).min().unwrap_or(i32::MAX)
                            > final_time
                    {
                        break 'find_conflicts;
                    }
                }

                poi.vehicles_since.retain(|(v, _)| v != v_idx);
            }

            // Enter
            if let Some(enter_poi) = curr_poi {
                let poi: &mut PoiState = pois.iter_mut().find(|p| p.task == enter_poi).unwrap();

                if poi.vehicles_since.iter().any(|(v, _)| *v == *v_idx) {
                    panic!(
                        "vehicle {} last_reward {:?} curr_reward {:?} poi_state {:?}",
                        v_idx, vehicle_poi[*v_idx as usize], curr_poi, poi
                    );
                }
                poi.vehicles_since.push((*v_idx, curr_time));
            }
        }

        vehicle_poi[*v_idx as usize] = curr_poi;

        // Advance the path_idx
        *path_idx += 1;
        if *path_idx as usize + 1 >= world.solutions[*v_idx as usize].path.len() {
            vehicle_path_idx.remove(0);
        } else {
            let mut rank = 0;
            while rank + 1 < vehicle_path_idx.len() && {
                let (v1, p1) = vehicle_path_idx[rank];
                let n1 = world.solutions[v1 as usize].path[p1 as usize];
                let t1 = world.nodes[n1 as usize].state.time;
                let (v2, p2) = vehicle_path_idx[rank + 1];
                let n2 = world.solutions[v2 as usize].path[p2 as usize];
                let t2 = world.nodes[n2 as usize].state.time;
                t1 > t2
            } {
                vehicle_path_idx.swap(rank, rank + 1);
                rank += 1;
            }
        }
    }

    info!("Found {} conflicts:", conflicts.len());
    for c in conflicts.iter() {
        info!("  - {:?}", c);
    }

    conflicts.into_iter().max_by_key(|conflict| {
        ordered_float::OrderedFloat(
            (final_time - conflict.time.0) as f32
                * world
                    .problem
                    .pois
                    .iter()
                    .find(|poi| poi.task_ref == conflict.poi)
                    .unwrap()
                    .reward_rate
                * conflict.vehicles.len() as f32,
        )
    })
}

fn generate_constraint(
    world: &World,
    conflict: &Conflict,
    vehicle: u32,
    start_time: i32,
    mut f: impl FnMut(Constraint),
) {
    for (node_idx, node) in world
        .nodes
        .iter()
        .enumerate()
        .filter(|(_, n)| n.state.loc.poi().is_some())
        .skip_while(|(_, n)| n.state.time < start_time)
        .take_while(|(_, n)| n.state.time < conflict.time.1)
    {
        for (edge_idx, edge) in node.outgoing.iter().enumerate() {
            // Filter edges that perform the task at POI.
            if world.nodes[edge.0 as usize].state.loc != node.state.loc {
                continue;
            }

            if node.state.loc.poi() != Some(conflict.poi) {
                f(Constraint {
                    vehicle,
                    node_idx: node_idx as u32,
                    edge_idx: edge_idx as u32,
                });
            } else {
                // The *other* vehicles are not allowed here.
                for (other_vehicle, _) in conflict.vehicles.iter().copied() {
                    if other_vehicle != vehicle {
                        f(Constraint {
                            vehicle: other_vehicle,
                            node_idx: node_idx as u32,
                            edge_idx: edge_idx as u32,
                        });
                    }
                }
            }
        }
    }
}

fn branch_on_conflict(world: &mut World) -> Option<(Conflict, Vec<Branch>)> {
    let _p = hprof::enter("branching");
    debug!(
        "checking conflicts with vehicle costs {:?}",
        world.solutions.iter().map(|s| s.cost).collect::<Vec<_>>()
    );
    let conflict: Conflict = find_conflict(world)?;
    // Then generate branches of constraints
    info!("solving conflict {:?}", conflict);
    let branches = conflict
        .vehicles
        .iter()
        .map(|(v, t)| {
            let mut cs = Vec::new();
            generate_constraint(world, &conflict, *v, *t, |c| {
                if !world.constraints.contains(&c) {
                    cs.push(c);
                }
            });

            let lb = world.temporary_constraints(&cs);
            Branch {
                constraints: cs,
                lb,
            }
        })
        .collect::<Vec<_>>();
    Some((conflict, branches))
}

// pub fn gbfs_deconflict_solve(problem_str: &str, time_horizon: f64, time_scale: i32) -> String {
//     match catch_unwind(|| _gbfs_deconflict_solve(problem_str, time_horizon, time_scale)) {
//         Ok(s) => s,
//         Err(e) => {
//             println!("ERROR");
//             "eRROR".to_string()
//         }
//     }
// }

// #[pyfunction]
pub fn gbfs_deconflict_solve(problem_str: &str, time_horizon: f32, time_scale: i32) -> String {
    // let _ = env_logger::try_init();
    let _p = hprof::enter("gbfs solve");
    std::fs::write("gbfs_problem.json", problem_str).unwrap();
    let problem: Problem = serde_json::from_str(problem_str).unwrap();
    debug!("new world");
    let mut world = World::new(&problem, time_horizon, time_scale);
    debug!("new world done");

    let mut iteration = 0;
    let mut solutions: Vec<(serde_json::Value, serde_json::Value)> = Vec::new();
    'gbfs: loop {
        if iteration >= 800 {
            panic!("gbfs_deconflict_solve did not converge");
        }

        info!(
            "gbfs iter {} obj {}",
            iteration,
            world.solutions.iter().map(|s| s.cost).sum::<f32>()
        );

        let json = serde_json::Value::Array(
            world
                .solutions
                .iter()
                .map(|s| {
                    serde_json::Value::Array(
                        s.path
                            .iter()
                            .map(|n| {
                                json!({
                                    "time": world.nodes[*n as usize].state.time,
                                    "loc": "TODO",
                                })
                            })
                            .collect(),
                    )
                })
                .collect(),
        );
        solutions.push((json, serde_json::Value::Null));

        // Greedy best-first search
        if let Some((conflict, branches)) = branch_on_conflict(&mut world) {
            solutions.last_mut().unwrap().1 = json!({
                "vehicles": conflict.vehicles.iter().map(|(v,_)| *v).collect::<Vec<_>>(),
                "start_time": conflict.time.0,
                "end_time": conflict.time.1,
                "poi": conflict.poi,
            });
            if branches.is_empty() {
                panic!("unexpected infeasible situation");
            }

            let best = branches
                .into_iter()
                .min_by_key(|b| ordered_float::OrderedFloat(b.lb))
                .unwrap();

            debug!(
                "Constraints {}+{}",
                world.constraints.len(),
                best.constraints.len()
            );
            info!("ADDING {} PERMANENT CONSTRAINTS", best.constraints.len());
            world.permanent_constraints(best.constraints.iter().copied());
            iteration += 1;
        } else {
            break 'gbfs;
        }
    }

    let solutions = solutions
        .into_iter()
        .map(|(sol, confl)| json!({"solution": sol, "conflict": confl}))
        .collect::<Vec<_>>();

    std::fs::write(
        "gbfs_log.json",
        serde_json::to_string_pretty(&solutions).unwrap(),
    )
    .unwrap();

    drop(_p);
    hprof::profiler().print_timing();
    // info!("{:#?}", problem);
    // info!("rust");
    // "asdf".to_string()

    serde_json::to_string(solutions.last().unwrap()).unwrap()
}
