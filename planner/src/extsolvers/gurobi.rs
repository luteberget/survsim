#![cfg(feature = "gurobi")]

use core::f64;
use std::cell::RefCell;

use super::LPSolver;
use grb::{expr::LinExpr, prelude::*};

thread_local! {
static GLOBAL_GUROBI_ENV :RefCell<Option<grb::Env>> = const { RefCell::new(None) };
}

pub struct GurobiSolver {
    grb: grb::Model,
    added_vars: Vec<grb::Var>,
}

impl LPSolver for GurobiSolver {
    type Var = grb::Var;

    fn new() -> Self {
        let mut grb = GLOBAL_GUROBI_ENV.with_borrow_mut(|e| {
            if e.is_none() {
                *e = Some(grb::Env::new("").unwrap());
            }
            let env = e.as_ref().unwrap();
            grb::Model::with_env("", env).unwrap()
        });
        grb.set_param(grb::param::OutputFlag, 0).unwrap();
        Self {
            grb,
            added_vars: Vec::new(),
        }
    }

    fn add_var(&mut self, cost: f64) -> Self::Var {
        let model = &mut self.grb;
        let var = add_ctsvar!(model, obj: cost, bounds: ..).unwrap();
        self.added_vars.push(var);
        var
    }

    fn set_binary(&mut self, var: Self::Var) {
        self.grb
            .set_obj_attr(grb::attr::VType, &var, VarType::Binary)
            .unwrap();
    }

    fn set_bounds(&mut self, var: Self::Var, lower: f64, upper: f64) {
        self.grb.update().unwrap();
        // println!("Setting lb {:?} to {:?} {:?}", var, lower, upper);
        self.grb.set_obj_attr(grb::attr::LB, &var, lower).unwrap();
        self.grb.set_obj_attr(grb::attr::UB, &var, upper).unwrap();

        if lower == upper {
            self.grb
                .set_obj_attr(grb::attr::IISLBForce, &var, 1)
                .unwrap();
            self.grb
                .set_obj_attr(grb::attr::IISUBForce, &var, 1)
                .unwrap();
        }
    }

    fn add_constraint(&mut self, lb: f64, ub: f64, idxs: &[Self::Var], coeffs: &[f64]) {
        let mut expr = LinExpr::new();
        for (v, c) in idxs.iter().zip(coeffs.iter()) {
            expr.add_term(*c, *v);
        }
        if lb == ub {
            self.grb.add_constr("", c!(expr == lb)).unwrap();
        } else if lb == -self.inf() {
            assert!(ub != self.inf());
            self.grb.add_constr("", c!(expr <= ub)).unwrap();
        } else if ub == self.inf() {
            assert!(lb != -self.inf());
            self.grb.add_constr("", c!(expr >= lb)).unwrap();
        } else {
            panic!("range constraints not supported");
            // self.grb
            //     .add_range(
            //         "",
            //         grb::constr::RangeExpr {
            //             expr: grb::Expr::from(expr),
            //             ub,
            //             lb,
            //         },
            //     )
            //     .unwrap();
        }
    }

    fn set_time_limit(&mut self, seconds: f64) {
        self.grb.set_param(grb::param::TimeLimit, seconds).unwrap();
    }

    fn optimize(&mut self) -> Option<(f64, f64, Vec<f64>)> {
        self.grb.optimize().unwrap();
        match self.grb.status().unwrap() {
            Status::Optimal
            | Status::CutOff
            | Status::IterationLimit
            | Status::NodeLimit
            | Status::TimeLimit
            | Status::SolutionLimit => {
                let bound = self.grb.get_attr(attr::ObjBound).unwrap();
                let obj = self.grb.get_attr(attr::ObjVal).unwrap();

                let sol = self
                    .grb
                    .get_obj_attr_batch(grb::attr::X, self.added_vars.iter().cloned())
                    .unwrap();

                Some((obj, bound, sol))
            }
            _ => {
                // self.grb.compute_iis().unwrap();
                // self.grb.write("iis.ilp").unwrap();
                None
            }
        }
    }

    fn inf(&self) -> f64 {
        f64::INFINITY
    }

    fn num_vars(&self) -> usize {
        self.grb.get_vars().unwrap().len()
    }

    fn write_model(&mut self) {
        self.grb.write("test_grb.lp").unwrap();
        self.grb.write("test_grb.mps").unwrap();
    }

    fn set_partial_solution(&mut self, idxs: &[Self::Var], values: &[f64]) {
        for (var, val) in idxs.iter().zip(values.iter()) {
            self.grb.set_obj_attr(grb::attr::Start, var, *val).unwrap();
        }
    }

    fn set_verbose(&mut self) {
        self.grb.set_param(grb::param::OutputFlag, 1).unwrap();
    }
}
