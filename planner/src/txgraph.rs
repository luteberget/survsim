use std::collections::HashMap;

use log::{debug, trace};
use survsim_structs::{
    problem::{Dist, Problem},
    report::Location,
};
use tinyvec::TinyVec;

use crate::round_time;

#[derive(Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct State {
    pub loc: Location,
    pub time: i32,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct EdgeData {
    pub cost: f32,
    pub batt: f32,
}

#[derive(Debug, Clone)]
pub struct Node {
    pub state: State,
    pub outgoing: TinyVec<[(u32, EdgeData, f32); 10]>,
}

pub fn production_edge(nodes: &[Node], node: usize) -> Option<usize> {
    match nodes[node].state.loc {
        Location::Task(task_ref) => nodes[node]
            .outgoing
            .iter()
            .enumerate()
            .filter(|&(_e_idx, (n2, _, _))| {
                nodes[*n2 as usize].state.loc == Location::Task(task_ref)
            })
            .map(|(e_idx, _)| (e_idx))
            .next(),
        _ => None,
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct Constraint {
    pub vehicle: u32,
    pub node_idx: u32,
    pub edge_idx: u32,
}

fn succ(
    problem: &Problem,
    dist_map: impl Fn(&(Location, Location)) -> Dist,
    time_horison: f32,
    time_scale: i32,
    state: &State,
    mut f: impl FnMut(State, EdgeData),
) {
    let air_time_cost = 1e-3 * (1.0 + 1.0 * state.time as f32 / time_horison);
    match state.loc {
        Location::SinkNode => {
            // Stay on ground
            // f(
            //     State {
            //         loc: Location::SinkNode,
            //         time: round_time(state.time as f32 + time_scale as f32, time_scale),
            //     },
            //     EdgeData {
            //         cost: 0.0,
            //         batt: 0.0,
            //     },
            // );
        }
        Location::Base => {
            // Stay on ground
            f(
                State {
                    loc: Location::Base,
                    time: round_time(state.time as f32 + time_scale as f32, time_scale),
                },
                EdgeData {
                    cost: 0.0,
                    batt: 0.0,
                },
            );

            // Terminate doing nothing
            f(
                State {
                    loc: Location::SinkNode,
                    time: round_time(state.time as f32 + time_scale as f32, time_scale),
                },
                EdgeData {
                    cost: 0.0,
                    batt: 0.0,
                },
            );

            // Go to pois
            for poi in problem.pois.iter() {
                let dist_edge = &(state.loc, Location::Task(poi.task_ref));
                let dist = dist_map(dist_edge);
                f(
                    State {
                        loc: Location::Task(poi.task_ref),
                        time: round_time(
                            state.time as f32 + (dist.dt).max(time_scale as f32),
                            time_scale,
                        ),
                    },
                    EdgeData {
                        cost: air_time_cost * (dist.dt).max(time_scale as f32),
                        batt: dist.d_batt,
                    },
                )
            }
        }

        Location::DroneInitial(_) => {
            for poi in problem.pois.iter() {
                let dist_edge = &(state.loc, Location::Task(poi.task_ref));
                let dist = dist_map(dist_edge);
                f(
                    State {
                        loc: Location::Task(poi.task_ref),
                        time: round_time(
                            state.time as f32 + (dist.dt).max(time_scale as f32),
                            time_scale,
                        ),
                    },
                    EdgeData {
                        cost: air_time_cost * (dist.dt).max(time_scale as f32),
                        batt: dist.d_batt,
                    },
                )
            }

            // Go to base
            let dist_edge = &(state.loc, Location::Base);
            let dist = dist_map(dist_edge);
            f(
                State {
                    loc: Location::SinkNode,
                    time: round_time(
                        state.time as f32 + (dist.dt).max(time_scale as f32),
                        time_scale,
                    ),
                },
                EdgeData {
                    cost: air_time_cost * (dist.dt).max(time_scale as f32),
                    batt: dist.d_batt,
                },
            );
        }

        Location::Task(curr_task) => {
            // Go to another poi
            for poi in problem.pois.iter() {
                if poi.task_ref == curr_task {
                    // Produce reward
                    f(
                        State {
                            loc: Location::Task(curr_task),
                            time: round_time(state.time as f32 + time_scale as f32, time_scale),
                        },
                        EdgeData {
                            cost: -(time_scale as f32) * poi.reward_rate
                                + air_time_cost * time_scale as f32,
                            batt: poi.battery_rate * time_scale as f32,
                        },
                    );
                } else {
                    // Go to another poi
                    let dist_edge = &(state.loc, Location::Task(poi.task_ref));
                    let dist = dist_map(dist_edge);
                    f(
                        State {
                            loc: Location::Task(poi.task_ref),
                            time: round_time(
                                state.time as f32 + (dist.dt).max(time_scale as f32),
                                time_scale,
                            ),
                        },
                        EdgeData {
                            cost: air_time_cost * (dist.dt).max(time_scale as f32),
                            batt: dist.d_batt,
                        },
                    );
                }
            }

            // Go to base
            let dist_edge = &(state.loc, Location::Base);
            let dist = dist_map(dist_edge);
            f(
                State {
                    loc: Location::SinkNode,
                    time: round_time(
                        state.time as f32 + (dist.dt).max(time_scale as f32),
                        time_scale,
                    ),
                },
                EdgeData {
                    cost: air_time_cost * (dist.dt).max(time_scale as f32),
                    batt: dist.d_batt,
                },
            );
        }
    }
}

pub fn build_graph(
    problem: &Problem,
    time_horizon: f32,
    time_scale: i32,
) -> (u32, Vec<u32>, Vec<Node>) {
    let _p_graph = hprof::enter("mk_graph");
    // Create the graph
    let start_states = problem
        .vehicles
        .iter()
        .enumerate()
        .map(|(v_idx, v)| {
            assert!(v.start_airborne || v.start_time.abs() < 1e-3);
            State {
                loc: if v.start_airborne {
                    Location::DroneInitial(v_idx)
                } else {
                    Location::Base
                },
                time: round_time(v.start_time, time_scale),
            }
        })
        .collect::<Vec<_>>();

    let mut queue: Vec<State> = Default::default();
    let mut state_n_incoming: HashMap<State, u32> = Default::default();
    let mut state_outgoing: HashMap<State, Vec<(State, EdgeData)>> = Default::default();

    for s in start_states.iter().chain(std::iter::once(&State {
        loc: Location::Base,
        time: 0,
    })) {
        state_n_incoming.entry(*s).or_default();
        state_outgoing.entry(*s).or_default();
        if !queue.contains(s) {
            queue.push(*s);
        }
    }

    let mut dist_map: HashMap<(Location, Location), Dist> = Default::default();
    for ((a, b), d) in problem.distances.iter() {
        dist_map.insert((*a, *b), *d);
        dist_map.insert((*b, *a), *d);
    }

    while let Some(state) = queue.pop() {
        // println!("txgraph frmo state {:?}", state);
        succ(
            problem,
            |e| *dist_map.get(e).unwrap(),
            time_horizon,
            time_scale,
            &state,
            |next_state, edge_data| {
                if next_state.time > round_time(time_horizon, time_scale) {
                    return;
                }
                state_outgoing
                    .get_mut(&state)
                    .unwrap()
                    .push((next_state, edge_data));

                // Does the next state exist?
                assert!(
                    state_outgoing.contains_key(&next_state)
                        == state_n_incoming.contains_key(&next_state)
                );
                if !state_outgoing.contains_key(&next_state) {
                    state_outgoing.entry(next_state).or_default();
                    queue.push(next_state);
                }
                *state_n_incoming.entry(next_state).or_default() += 1;
            },
        );
    }

    let mut queue: Vec<State> = state_n_incoming
        .iter()
        .filter(|&(_k, n)| (*n == 0))
        .map(|(k, _n)| *k)
        .collect();

    queue.sort_by_key(|s| -s.time);

    let mut nodes: Vec<Node> = Default::default();
    let mut node_idxs: HashMap<State, u32> = Default::default();
    // let mut final_node: Option<(u32, i32)> = None;

    while let Some(state) = queue.pop() {
        node_idxs.insert(state, nodes.len() as u32);
        nodes.push(Node {
            outgoing: Default::default(),
            state,
        });

        // let is_final = (state.loc == Location::Base || state.loc == Location::SinkNode)
        //     && state.time > final_node.as_ref().map(|(_, t)| *t).unwrap_or(i32::MIN);
        // if is_final {
        //     final_node = Some((node_idxs[&state], state.time));
        // }

        for (next_state, _) in state_outgoing[&state].iter() {
            let deg = state_n_incoming.get_mut(next_state).unwrap();
            *deg -= 1;
            if *deg == 0 {
                let mut new_idx = queue.len();
                queue.push(*next_state);
                while new_idx > 0 && queue[new_idx].time > queue[new_idx - 1].time {
                    queue.swap(new_idx, new_idx - 1);
                    new_idx -= 1;
                }
            }
        }
    }

    // Add edges
    for (s1, outg) in state_outgoing.into_iter() {
        let i1 = node_idxs[&s1];
        for (s2, ed) in outg.into_iter() {
            nodes[i1 as usize].outgoing.push((node_idxs[&s2], ed, 0.0));
        }
    }

    debug!(
        "Constructed graph with {} nodes, {} edges",
        nodes.len(),
        nodes.iter().map(|n| n.outgoing.len()).sum::<usize>()
    );

    let vehicle_start_nodes = start_states
        .iter()
        .map(|x| node_idxs[x])
        .collect::<Vec<_>>();

    let default_node = node_idxs[&State {
        loc: Location::Base,
        time: 0,
    }];

    debug!(
        "Start nodes {:?}",
        vehicle_start_nodes
            .iter()
            .map(|n| nodes[*n as usize].state)
            .collect::<Vec<_>>()
    );

    // Should be topologically ordered
    for (i, n1) in nodes.iter().enumerate() {
        for (j, _, _) in n1.outgoing.iter() {
            trace!(
                " {} {:?} --  {}  {:?} ",
                i,
                n1.state,
                j,
                nodes[*j as usize].state
            );
            assert!(i < (*j as usize));
        }
    }
    // Should also be ordered by time
    for (i, n1) in nodes.iter().enumerate() {
        for (j, n2) in nodes.iter().skip(i + 1).enumerate() {
            if n1.state.time > n2.state.time {
                panic!("i={} n1={:?} j={} n2={:?}", i, n1.state, j, n2.state);
            }
        }
    }

    debug_assert!(nodes.last().map(|n| n.state.time) == nodes.iter().map(|n| n.state.time).max());

    (default_node, vehicle_start_nodes, nodes)
}
