use std::ffi::{c_void, CStr};
use survsim_structs::{
    plan::Plan,
    problem::{Poi, Problem},
};

#[test]
pub fn test() {
    let x = solve(&Problem {
        battery_capacity: 0.0,
        t_takeoff: 0.0,
        vehicles: vec![],
        pois: vec![],
        distances: vec![],
    });
    println!("{:?}", x);
}

pub fn solve(problem: &Problem) -> Plan {
    let ts = vec![0, 30, 60, 90, 120];

    let mut edges: Vec<bool> = Vec::new();

    let mut rmp = LPInstance::new();

    let capacity_vehicle = ts
        .iter()
        .map(|_t| rmp.add_empty_constr(0.0, 6.0))
        .collect::<Vec<_>>();

    let edge_capacity = edges
        .iter()
        .enumerate()
        .filter(|(i, x)| **x)
        .map(|x| rmp.add_empty_constr(0.0, 1.0))
        .collect::<Vec<_>>();

    let rmp_columns: Vec<usize> = Vec::new();

    #[allow(clippy::never_loop)]
    loop {
        rmp.optimize();
        // Optimize the RMP
        panic!("ok");
    }

    todo!()
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

    pub fn add_empty_constr(&mut self, lb: f64, ub: f64) -> usize {
        let new_row_idx = unsafe { highs_sys::Highs_getNumRow(self.ptr) } as usize;
        unsafe { highs_sys::Highs_addRow(self.ptr, lb, ub, 0, std::ptr::null(), std::ptr::null()) };
        new_row_idx
    }

    pub fn optimize(&mut self) {
        let retval = unsafe { highs_sys::Highs_run(self.ptr) };
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

        println!(
            "primal {} dual {}",
            primal_solution_status, dual_solution_status
        );

        let n = std::ptr::null_mut();
        unsafe { highs_sys::Highs_getSolution(self.ptr, n, n, n, n) };

        println!("getsolution ok");
    }
}
