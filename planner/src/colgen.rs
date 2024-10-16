use ordered_float::OrderedFloat;
use std::{
    collections::{BTreeSet, HashSet},
    ffi::{c_void, CStr},
};
use survsim_structs::{plan::Plan, problem::Problem, report::Location};
use tinyvec::TinyVec;

use crate::{
    shortest_path::plan_vehicle,
    txgraph::{self, production_edge, Constraint},
    VehicleSolution,
};

#[test]
pub fn test() {
    let _ = env_logger::try_init();
    let problem =
        serde_json::from_str(&std::fs::read_to_string("../tiny_problem.json").unwrap()).unwrap();
    let x = solve(&problem, true);
    hprof::profiler().print_timing();
    println!("{:?}", x);
}

pub fn solve(problem: &Problem, use_heuristic: bool) -> Plan {
    let _p = hprof::enter("colgen solve");
    let (vehicle_start_nodes, mut nodes) = txgraph::build_graph(problem, 1.5 * 3600., 30);

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
    let time_steps = time_steps;

    let mut rmp = LPInstance::new();
    let mut shadow_prices: Vec<f64> = Default::default();

    let veh_cstrs = time_steps
        .iter()
        .map(|_t| rmp.add_empty_constr(0.0, problem.vehicles.len() as f64))
        .collect::<Vec<_>>();
    shadow_prices.extend(veh_cstrs.iter().map(|_| 0.0));
    let mut time_cost: Vec<(i32, f32)> = time_steps.iter().map(|t| (*t as i32, 0.0)).collect();

    let production_capacity = (0..nodes.len())
        .map(|i| production_edge(&nodes, i).map(|e| (e, rmp.add_empty_constr(0.0, 1.0))))
        .collect::<Vec<Option<_>>>();
    shadow_prices.extend(
        production_capacity
            .iter()
            .filter(|x| x.is_some())
            .map(|_| 0.0),
    );

    let mut last_fix_with_cols = 0;

    #[allow(unused)]
    #[derive(Debug)]
    struct MasterColumn {
        col_idx: u32,
        // cost: f32,
        solution: VehicleSolution,
    }
    let mut rmp_columns: Vec<MasterColumn> = Vec::new();
    // let mut incumbent_values: Vec<f64> = Default::default();
    let constraints: BTreeSet<Constraint> = BTreeSet::new();
    let mut label_buf: Vec<TinyVec<[crate::shortest_path::Label; 20]>> =
        nodes.iter().map(|_| Default::default()).collect();

    let mut idxs_buf: Vec<i32> = Default::default();
    let mut coeffs_buf: Vec<f64> = Default::default();

    let mut solution_buf: Vec<f64> = Default::default();

    #[allow(clippy::never_loop)]
    loop {
        // Optimize the RMP
        println!("Solving RMP");
        // rmp.write_model();
        // let mut temp = (0..rmp_columns.len()).map(|_| 0.0f64).collect::<Vec<_>>();
        let objective_value = rmp.optimize(&mut [], &mut shadow_prices).unwrap();
        println!(" - objective {:.2}", objective_value);
        // println!("  variable values {:?}", temp);
        // println!("  shadow prices values {:?}", shadow_prices);

        if use_heuristic {
            if rmp_columns.len() >= last_fix_with_cols + 25 {
                while solution_buf.len() < rmp_columns.len() {
                    solution_buf.push(Default::default());
                }

                rmp.get_solution(&mut solution_buf);
                assert!(solution_buf.iter().sum::<f64>() >= 1.0 - 1e-3);
                println!("sol {:?}", solution_buf);

                let (selected_col, _) = rmp_columns
                    .iter()
                    .zip(solution_buf.iter())
                    .filter(|(_, x)| **x >= 1e-3)
                    .max_by_key(|(c, x)| {
                        OrderedFloat(
                            **x as f32
                                - (nodes[c.solution.path[0] as usize].state.time as f32) / 1800.0,
                        )
                    })
                    .unwrap();

                println!(
                    "wil fix {} t={}",
                    selected_col.col_idx,
                    (nodes[selected_col.solution.path[0] as usize].state.time as f32)
                );

                panic!();
            }
        }

        let mut n_nonzero_prices = 0;
        let mut prod_cost_sum = 0.0f32;
        // Update edge shadow prices
        for ((n, (e, _row)), p) in production_capacity
            .iter()
            .enumerate()
            .filter_map(|(n, x)| x.map(|e| (n, e)))
            .zip(shadow_prices.iter().skip(veh_cstrs.len()))
        {
            nodes[n].outgoing[e].2 = -(*p as f32);
            if p.abs() > 1e-3 {
                n_nonzero_prices += 1;
            }
            prod_cost_sum += *p as f32;
        }
        // Update vehicle shadow prices
        for ((_, x), p) in time_cost.iter_mut().zip(shadow_prices.iter()) {
            *x = -(*p as f32);
            if p.abs() > 1e-3 {
                n_nonzero_prices += 1;
            }
        }
        println!(" {} nonzero prices", n_nonzero_prices);
        println!(
            "   time cost sum {} {:?}",
            time_cost.iter().map(|(_, c)| *c).sum::<f32>(),
            &time_cost[..10]
        );
        println!("   prod cost sum {}", prod_cost_sum);

        let mut added_column = false;
        // Solve the subproblem of finding a new vehicle path
        if let Some(solution) = plan_vehicle(
            u32::MAX,
            &nodes,
            &vehicle_start_nodes,
            0.0,
            &mut label_buf,
            &constraints,
            &time_cost,
        ) {
            const COLGEN_COST_TOL: f32 = 5.0;
            if solution.cost_including_shadow_price < -COLGEN_COST_TOL {
                // Create new column
                idxs_buf.clear();
                coeffs_buf.clear();

                // Record vehicle usage
                let mut curr_t = time_steps.iter().enumerate().peekable();
                for (s1, s2) in solution.states(&nodes).zip(solution.states(&nodes).skip(1)) {
                    while s1.time > curr_t.peek().map(|(_, t)| **t).unwrap() {
                        curr_t.next();
                    }
                    let on_ground = (s1.loc == Location::Base && s2.loc == Location::Base)
                        || (s1.loc == Location::SinkNode && s2.loc == Location::SinkNode);

                    if !on_ground {
                        while s2.time > curr_t.peek().map(|(_, t)| **t).unwrap() {
                            idxs_buf.push(veh_cstrs[curr_t.peek().unwrap().0] as i32);
                            coeffs_buf.push(1.0);
                            curr_t.next();
                        }
                    }
                }

                // for s in plan.states(&nodes) {
                //     println!("  {:?}", s);
                // }

                let n_timeslots = idxs_buf.len();
                let t1 = idxs_buf.iter().next().map(|i| time_steps[*i as usize]);
                let t2 = idxs_buf.last().map(|i| time_steps[*i as usize]);

                // Record production edges
                for (n1, n2) in solution.path.iter().zip(solution.path.iter().skip(1)) {
                    if let Some((e_idx, row_idx)) = &production_capacity[*n1 as usize] {
                        if nodes[*n1 as usize].outgoing[*e_idx].0 == *n2 {
                            assert!(nodes[*n1 as usize].state.loc == nodes[*n2 as usize].state.loc);
                            idxs_buf.push(*row_idx as i32);
                            coeffs_buf.push(1.0);
                        }
                    }
                }

                let n_production_edges = idxs_buf.len() - n_timeslots;
                println!("  adding col c={}/sc={} with {} time slots t={:?} --> t={:?} and {} production edges", solution.cost, solution.cost_including_shadow_price, n_timeslots, t1,t2, n_production_edges);

                let col_idx = rmp.add_column(solution.cost as f64, &idxs_buf, &coeffs_buf);
                rmp_columns.push(MasterColumn {
                    col_idx,
                    // cost: plan.cost,
                    solution,
                });

                added_column = true;
                // if rmp_columns.len() >= 1000 {
                //     panic!("added {} columns", rmp_columns.len())
                // }
            } else {
                println!(
                    "BNP node converged (sc={})",
                    solution.cost_including_shadow_price
                );
            }
        } else {
            println!("Warning : shortest path problem was infeasible");
        };

        if !added_column {
            let _p2 = hprof::enter("price and branch IP");
            println!("solving price and branch");
            for c in rmp_columns.iter() {
                rmp.set_binary(c.col_idx as i32);
            }
            let mut var_values = vec![0.0; rmp_columns.len()];
            let obj = rmp.optimize(&mut var_values, &mut []).unwrap();
            println!("solved binary program with obj val {:.2}", obj);
            assert!(var_values
                .iter()
                .all(|x| (x.abs() < 1e-4) || (1.0 - x).abs() < 1e-4));
            let selected_cols = rmp_columns
                .iter()
                .filter(|x| var_values[x.col_idx as usize] > 0.5)
                .collect::<Vec<_>>();
            for v in selected_cols.iter() {
                println!("selected {:?}", v);
            }

            drop(_p2);
            hprof::profiler().print_timing();

            panic!("finisehd");
        }
    }
}

struct LPInstance {
    ptr: *mut c_void,
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

    pub fn optimize(&mut self, var_value_out: &mut [f64], row_dual_out: &mut [f64]) -> Option<f64> {
        let _p = hprof::enter("optimize");
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

        println!("getsolution ok");

        Some(objective_value)
    }
}
