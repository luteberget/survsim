// #![deny(clippy::print_stdout)]
use std::collections::BTreeSet;

use log::{debug, trace};
use ordered_float::OrderedFloat;
use survsim_structs::report::Location;
use tinyvec::TinyVec;

use crate::{
    txgraph::{Constraint, Node},
    VehicleSolution,
};

#[derive(Default, Debug)]
pub struct Label {
    cost: f32,
    remaining_battery: f32,
    prev_node: u32,
}

fn label_dominates(a: &Label, b: &Label) -> bool {
    const APPROXIMATE_DOMINANCE: bool = false;

    // Correct dominance:

    if !APPROXIMATE_DOMINANCE {
        a.cost <= b.cost && a.remaining_battery >= b.remaining_battery
    } else {
        // Approximate dominance:

        if a.cost <= b.cost && a.remaining_battery >= b.remaining_battery {
            return true;
        }

        if a.cost < b.cost && a.remaining_battery * 1.09 >= b.remaining_battery {
            return true;
        }

        false
    }
    // || (a.cost < b.cost && a.remaining_battery + 0.02 >= b.remaining_battery)
}

fn remove_dominated(front: &mut tinyvec::TinyVec<[Label; 20]>, new_elem: &Label) -> bool {
    // println!("Remove dominated len={} rc{} batt{}",  front.len(), new_elem.reduced_cost, new_elem.remaining_battery);
    for (index, elem) in front.iter().enumerate() {
        if label_dominates(elem, new_elem) {
            // println!("  DOM dominated by {} {}", elem.cost, elem.remaining_battery);
            if index > 0 {
                front.swap(index, index - 1);
            }
            return false;
        } else if label_dominates(new_elem, elem) {
            // println!("  DOM remove {} {}", front[index].cost, front[index].remaining_battery);
            front.swap_remove(index);
            {
                let mut index = index;
                while index < front.len() {
                    if label_dominates(new_elem, &front[index]) {
                        // println!("  DOM remove {} {}", front[index].cost, front[index].remaining_battery);
                        front.swap_remove(index);
                    } else {
                        index += 1;
                    }
                }
            }

            // println!("  len={}", front.len());
            return true;
        }
    }
    true
}

#[allow(clippy::too_many_arguments)]
pub fn plan_vehicle(
    vehicle: u32,
    nodes: &[Node],
    start_nodes: &[(u32, f32)],
    recharge_at_base: f32,
    label_buf: &mut [TinyVec<[Label; 20]>],
    constraints: &BTreeSet<Constraint>,
    time_steps: &[i32],
    time_cost: &[f32],
) -> Option<VehicleSolution> {
    #[cfg(feature="prof")]
    let _p = hprof::enter("plan_vehicle");
    #[cfg(feature="prof")]
    let _p0 = hprof::enter("plan init");
    assert!(nodes.len() == label_buf.len());
    // let final_time = nodes.last().map(|x| x.state.time);
    let range = Constraint {
        vehicle,
        node_idx: u32::MIN,
        edge_idx: u32::MIN,
    }..Constraint {
        vehicle,
        node_idx: u32::MAX,
        edge_idx: u32::MAX,
    };
    debug!(
        "Solving vehicle {} from nodes {:?} constraints {}",
        vehicle,
        start_nodes,
        constraints.range(range.clone()).count()
    );

    debug!(
        "CONSTRAINTS: {:?}",
        constraints.range(range.clone()).collect::<Vec<_>>()
    );
    debug!("TIME COST ARRAY {:?}", time_cost);
    let mut constraints = constraints.range(range).peekable();
    let mut time_cost_idx = 0;

    for n in label_buf.iter_mut() {
        n.clear();
    }

    for (start_node, start_battery) in start_nodes {
        label_buf[*start_node as usize].push(Label {
            cost: 0.0,
            // prev_label: u32::MAX,
            prev_node: u32::MAX,
            remaining_battery: *start_battery,
        });
        debug!(
            "Start node {:?} {:?}",
            nodes[*start_node as usize].state, label_buf[*start_node as usize]
        );
    }

    let mut best: Option<(f32, u32, u32)> = Default::default();

    let mut n_ops = 0;
    // Iterate all nodes

    #[cfg(feature="prof")]
    drop(_p0);
    for (src_node_idx, node) in nodes.iter().enumerate() {
        #[cfg(feature="prof")]
        let _pp = hprof::enter("node");
        trace!("Searching from node {} {:?}", src_node_idx, node.state);
        if
        /*Some(node.state.time) == final_time && */
        node.state.loc == Location::SinkNode {
            // This is a goal node.
            for (l, label) in label_buf[src_node_idx].iter().enumerate() {
                if label.cost < best.map(|(s, _, _)| s).unwrap_or(f32::INFINITY) {
                    best = Some((label.cost, src_node_idx as u32, l as u32));
                }
            }
        }

        for (edge_idx, (tgt_node_idx, edge_data, edge_shadow_price)) in
            node.outgoing.iter().enumerate()
        {
            let target_node = &nodes[*tgt_node_idx as usize];

            // Advance the constraint pointer
            while constraints
                .peek()
                .map(|c| (c.node_idx, c.edge_idx) < (src_node_idx as u32, edge_idx as u32))
                .unwrap_or(false)
            {
                constraints.next();
            }

            // Advance time cost pointer
            while time_steps[time_cost_idx] < node.state.time {
                time_cost_idx += 1;
            }

            let edge_on_ground = (node.state.loc == Location::Base
                || node.state.loc == Location::SinkNode)
                && (target_node.state.loc == Location::Base
                    || target_node.state.loc == Location::SinkNode);
            let edge_time_cost = if edge_on_ground {
                0.0
            } else {
                time_steps[time_cost_idx..]
                    .iter()
                    .zip(time_cost[time_cost_idx..].iter())
                    .take_while(|(t, _)| **t < target_node.state.time)
                    .map(|(_, c)| *c)
                    .sum()
            };

            if edge_shadow_price.is_infinite() || edge_time_cost.is_infinite() {
                continue;
            }

            if !edge_on_ground && node.state.time == 0 || node.state.time == 30 {
                // println!("edge time cost {:?} for {:?}", edge_time_cost, node.state);
            }

            for label_idx in 0..label_buf[src_node_idx].len() {
                trace!(" - LABEL {:?}", label_buf[src_node_idx][label_idx]);
                n_ops += 1;

                // // Check constraint
                // if let Some(c) = constraints.peek() {
                //     if (c.node_idx as usize) == src_node_idx && (c.edge_idx as usize) == edge_idx {
                //         debug!(
                //             "Cannot use v{} {:?} -- {:?}",
                //             vehicle,
                //             nodes[src_node_idx].state,
                //             nodes[nodes[src_node_idx].outgoing[c.edge_idx as usize].0 as usize]
                //                 .state
                //         );
                //         continue;
                //     }
                // }

                let prev_label = &label_buf[src_node_idx][label_idx];

                // Check battery level
                if prev_label.remaining_battery - edge_data.batt < 0.0 {
                    continue;
                }

                let remaining_battery = if nodes[*tgt_node_idx as usize].state.loc == Location::Base
                {
                    recharge_at_base.max(prev_label.remaining_battery - edge_data.batt)
                } else {
                    prev_label.remaining_battery - edge_data.batt
                };

                // Make new label
                let new_label = Label {
                    cost: prev_label.cost + edge_data.cost + edge_shadow_price + edge_time_cost,
                    remaining_battery,
                    prev_node: src_node_idx as u32,
                    // prev_label: label_idx as u32,
                };
                // debug!("adding {}-{}", src_node_idx, tgt_node_idx);

                let target_node_labels = &mut label_buf[*tgt_node_idx as usize];
                let is_pareto_optimal = remove_dominated(target_node_labels, &new_label);
                if is_pareto_optimal {
                    target_node_labels.push(new_label);
                }
            }
        }
    }
    #[cfg(feature="prof")]
    let _p2 = hprof::enter("reconstruct path");

    // The problem should never be infeasible.
    // println!("BEST {:?}", best);

    let (cost_including_shadow_price, mut node_idx, mut label_idx) = best?;
    let mut cost: f32 = 0.0;
    let mut path = Vec::new();
    loop {
        path.push(node_idx);
        trace!(
            "Adding node {} {:?}",
            node_idx,
            nodes[node_idx as usize].state
        );

        let label = &label_buf[node_idx as usize][label_idx as usize];
        if label.prev_node == u32::MAX {
            break;
        }

        let edge = nodes[label.prev_node as usize]
            .outgoing
            .iter()
            .filter(|(_, _, c)| !c.is_infinite())
            .find(|(n, _, _)| *n == node_idx)
            .unwrap();

        cost += edge.1.cost;

        let required_batt = if nodes[node_idx as usize].state.loc == Location::Base {
            0.0
        } else {
            label.remaining_battery
        } + edge.1.batt;

        // Tolerance on battery label matching.
        let required_batt = required_batt - (0.001 * required_batt).abs();

        trace!(
            "req_batt {} prevlabels {:?}",
            required_batt,
            label_buf[label.prev_node as usize]
        );

        let prev_label = label_buf[label.prev_node as usize]
            .iter()
            .enumerate()
            .filter(|(_, l)| l.remaining_battery >= required_batt)
            .min_by_key(|(_, l)| OrderedFloat(l.cost))
            .unwrap()
            .0 as u32;

        let label = &label_buf[node_idx as usize][label_idx as usize];
        (node_idx, label_idx) = (label.prev_node, prev_label);
    }
    path.reverse();

    debug!(
        "SOLVED v{} path {:?}",
        vehicle,
        path.iter()
            .map(|n| nodes[*n as usize].state)
            .collect::<Vec<_>>()
    );

    let n_labels = label_buf.iter().map(|l| l.len()).sum::<usize>();
    // drop(_p);
    // hprof::profiler().print_timing();
    debug!(
        "solved one with cost {},  {} labels {} ops",
        cost, n_labels, n_ops
    );

    Some(VehicleSolution {
        cost,
        cost_including_shadow_price,
        path,
    })
}
