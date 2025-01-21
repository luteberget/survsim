use std::collections::HashMap;

use survsim_structs::{
    backend::Task,
    plan::{Cond, Plan, PlanTask},
    problem::Problem,
    report::Location,
    TaskRef,
};

use crate::txgraph::{EdgeData, Node, State};

#[derive(Debug, Clone)]
pub struct BattCycPlan {
    pub cost: f32,
    pub path: Vec<u32>,
}

pub fn cyc_plan_info(plan: &BattCycPlan, nodes: &[Node]) -> String {
    let ((_start_path_idx, _end_path_idx), (start_time, end_time)) = cyc_start_end(plan, nodes);
    let prod_intervals = cyc_production_intervals(plan, nodes);

    format!(
        "Plan(nd={:?}, cost={:.2}, time=({},{}), prod={:?})",
        nodes[plan.path[0] as usize].state, plan.cost, start_time, end_time, prod_intervals
    )
}

pub fn cyc_start_end(plan: &BattCycPlan, nodes: &[Node]) -> ((usize, usize), (i32, i32)) {
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
    let path_end_time = nodes[plan.path[end_path_idx] as usize].state.time;

    let time_step = get_txgraph_time_step(nodes);
    let end_time = path_end_time + time_step;

    ((start_path_idx, end_path_idx), (start_time, end_time))
}

fn get_txgraph_time_step(nodes: &[Node]) -> i32 {
    let b1 = nodes
        .iter()
        .find(|n| n.state.loc == Location::Base)
        .unwrap();
    let b2 = b1
        .outgoing
        .iter()
        .map(|(n, _, _)| &nodes[*n as usize])
        .find(|n| n.state.loc == Location::Base)
        .unwrap();
    
    b2.state.time - b1.state.time
}

pub fn cyc_production_intervals(plan: &BattCycPlan, nodes: &[Node]) -> Vec<(TaskRef, i32, i32)> {
    let mut prod_intervals: Vec<(TaskRef, i32, i32)> = Vec::new();
    for (n1, n2) in plan.path.iter().zip(plan.path.iter().skip(1)) {
        let (s1, s2) = (&nodes[*n1 as usize].state, &nodes[*n2 as usize].state);
        if s1.loc.poi().is_some() {
            if prod_intervals.last().is_none()
                || prod_intervals.last().unwrap().0 != s1.loc.poi().unwrap()
                || prod_intervals.last().unwrap().2 != s1.time
            {
                prod_intervals.push((s1.loc.poi().unwrap(), s1.time, s1.time));
            }

            prod_intervals.last_mut().unwrap().2 = s1.time;
            if s2.loc == s1.loc {
                prod_intervals.last_mut().unwrap().2 = s2.time;
            }
        }

        //  && s1.loc == s2.loc {
        //     if let Some((_t, _t1, t2)) = prod_intervals
        //         .last_mut()
        //         .filter(|(t, _, _)| *t == s1.loc.poi().unwrap())
        //     {
        //         *t2 = s2.time;
        //     } else {
        //         prod_intervals.push((s1.loc.poi().unwrap(), s1.time, s2.time));
        //     }
        // }
    }
    prod_intervals
}

pub fn get_plan_edges_in_air(nodes: &[Node], path: &[u32], time: &[i32], mut f: impl FnMut(usize)) {
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
            while t_idx < time.len() && time[t_idx] <= s2.time {
                f(t_idx);
                t_idx += 1;
            }
        }
    }
}

pub fn get_plan_prod_nodes(nodes: &[Node], path: &[u32], mut f: impl FnMut(u32)) {
    // Record production edges
    for (n1, n2) in path.iter().zip(path.iter().skip(1)) {
        let (s1, s2) = (&nodes[*n1 as usize].state, &nodes[*n2 as usize].state);
        if s1.loc.poi().is_some() && s1.loc == s2.loc {
            f(*n1);
        }
    }
}

pub struct TxGraphPlan(pub Vec<Vec<(State, Option<EdgeData>)>>);

pub fn convert_batt_cyc_plan(
    problem: &Problem,
    nodes: &[Node],
    vehicle_start_nodes: &[(u32, f32)],
    components: Vec<BattCycPlan>,
) -> (Plan, TxGraphPlan) {
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

    let all_time_steps = {
        let steps = nodes
            .iter()
            .map(|n| n.state.time)
            .collect::<std::collections::HashSet<_>>();
        let mut steps = steps.into_iter().collect::<Vec<_>>();
        steps.sort();
        steps
    };

    let mut v_txplans: Vec<Vec<(State, Option<EdgeData>)>> = vehicle_start_nodes
        .iter()
        .map(|(n, _)| vec![(nodes[*n as usize].state, None)])
        .collect::<Vec<_>>();

    let n_prod_itervals = components
        .iter()
        .map(|x| cyc_production_intervals(x, nodes).len())
        .sum::<usize>();
    println!(
        "COUNT ------- {} batt cycles, {} production intervals   obj: {:.2}",
        components.len(),
        n_prod_itervals,
        components.iter().map(|c| c.cost).sum::<f32>(),
    );

    for plan_idx in start_time_order {
        let plan = &components[plan_idx];

        let ((start_path_idx, end_path_idx), (start_time, end_time)) =
            start_and_end_times[plan_idx];
        // println!("treating plan {:?}", plan);
        println!(" treating plan {}", cyc_plan_info(plan, nodes));

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

        assert!(v_txplans[v_idx].last().unwrap().0.time <= start_time);
        while v_txplans[v_idx].last().unwrap().0.time < start_time {
            assert!(v_txplans[v_idx].last().unwrap().0.loc == Location::Base);
            // find next time stamp
            let pos = all_time_steps
                .binary_search(&v_txplans[v_idx].last().unwrap().0.time)
                .unwrap();
            let next_time = all_time_steps[pos + 1];
            v_txplans[v_idx].push((
                State {
                    loc: Location::Base,
                    time: next_time,
                },
                // Base edges should be neg-inf batt and zero cost.
                Some(EdgeData {
                    batt: -problem.battery_capacity,
                    cost: 0.0,
                }),
            ));
        }

        for (n1, n2) in plan.path[start_path_idx..end_path_idx]
            .iter()
            .zip(plan.path[start_path_idx + 1..].iter())
        {
            // We should already be at n1.
            assert!(v_txplans[v_idx].last().unwrap().0 == nodes[*n1 as usize].state);
            let edge = nodes[*n1 as usize]
                .outgoing
                .iter()
                .find_map(|(n, d, _)| (*n == *n2).then_some(*d))
                .unwrap();
            v_txplans[v_idx].push((nodes[*n2 as usize].state, Some(edge)));

            // Rewrite sink to base
            {
                let vtx_last = v_txplans[v_idx].last_mut().unwrap();
                if vtx_last.0.loc == Location::SinkNode {
                    vtx_last.0.loc = Location::Base;
                }
            }

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

        // in the v_txplan, we add the recharge edge at the end of the plan (unless we are at the end of the time horizon)
        assert!(v_txplans[v_idx].last().unwrap().0.loc == Location::Base);
        if v_txplans[v_idx].last().unwrap().0.time < *all_time_steps.last().unwrap() {
            let pos = all_time_steps
                .binary_search(&v_txplans[v_idx].last().unwrap().0.time)
                .unwrap();
            let next_time = all_time_steps[pos + 1];

            v_txplans[v_idx].push((
                State {
                    loc: Location::Base,
                    time: next_time,
                },
                Some(EdgeData {
                    batt: -problem.battery_capacity,
                    cost: 0.0,
                }),
            ));
        }
    }

    // Extend the plans to the end of the plnning horizon
    let last_time = *all_time_steps.last().unwrap();
    for (v_idx, _v) in problem.vehicles.iter().enumerate() {
        assert!(v_txplans[v_idx].last().unwrap().0.time <= last_time);
        while v_txplans[v_idx].last().unwrap().0.time < last_time {
            assert!(v_txplans[v_idx].last().unwrap().0.loc == Location::Base);
            // find next time stamp
            let pos = all_time_steps
                .binary_search(&v_txplans[v_idx].last().unwrap().0.time)
                .unwrap();
            let next_time = all_time_steps[pos + 1];
            v_txplans[v_idx].push((
                State {
                    loc: Location::Base,
                    time: next_time,
                },
                Some(EdgeData {
                    batt: -problem.battery_capacity,
                    cost: 0.0,
                }),
            ));
        }
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

    (
        Plan {
            vehicle_tasks: v_plans
                .into_iter()
                .map(|x| x.into_iter().map(|(_, t)| t).collect())
                .collect(),
        },
        TxGraphPlan(v_txplans),
    )
}
