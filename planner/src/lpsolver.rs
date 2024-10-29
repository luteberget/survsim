use std::ffi::{c_void, CStr};

use crate::highs_status::{HighsModelStatus, HighsStatus};

pub struct LPInstance {
    ptr: *mut c_void,
}

impl Drop for LPInstance {
    fn drop(&mut self) {
        unsafe {
            highs_sys::Highs_destroy(self.ptr);
        }
    }
}

impl Default for LPInstance {
    fn default() -> Self {
        Self::new()
    }
}

impl LPInstance {
    pub fn new() -> Self {
        let ptr = unsafe { highs_sys::Highs_create() };
        unsafe {
            highs_sys::Highs_setStringOptionValue(
                ptr,
                CStr::from_bytes_with_nul("presolve\0".as_bytes())
                    .unwrap()
                    .as_ptr(),
                    CStr::from_bytes_with_nul("on\0".as_bytes())
                    .unwrap()
                    .as_ptr(),
            )
        };
        // unsafe {
        //     highs_sys::Highs_setStringOptionValue(
        //         ptr,
        //         CStr::from_bytes_with_nul("solver\0".as_bytes())
        //             .unwrap()
        //             .as_ptr(),
        //             CStr::from_bytes_with_nul("ipm\0".as_bytes())
        //             .unwrap()
        //             .as_ptr(),
        //     )
        // };
        unsafe {
            highs_sys::Highs_setBoolOptionValue(
                ptr,
                CStr::from_bytes_with_nul("output_flag\0".as_bytes())
                    .unwrap()
                    .as_ptr(),
                1,
            )
        };
        Self { ptr }
    }

    #[allow(unused)]
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

    pub fn inf(&self) -> f64 {
        unsafe { highs_sys::Highs_getInfinity(self.ptr) }
    }

    pub fn add_column(&mut self, cost: f64, idxs: &[i32], coeffs: &[f64]) -> u32 {
        let new_col_idx = unsafe { highs_sys::Highs_getNumCol(self.ptr) } as u32;
        let inf = self.inf();
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
        let status = HighsStatus::try_from(retval);
        assert!(status == Ok(HighsStatus::OK));
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
        unsafe {
            highs_sys::Highs_writeModel(
                self.ptr,
                CStr::from_bytes_with_nul("test.mps\0".as_bytes())
                    .unwrap()
                    .as_ptr(),
            )
        };
        println!("model saved");
    }

    pub fn get_solution(&mut self, var_value_out: &mut [f64]) {
        let num_cols = unsafe { highs_sys::Highs_getNumCol(self.ptr) } as usize;

        if !var_value_out.is_empty() {
            assert!(var_value_out.len() == num_cols);

            let null = std::ptr::null_mut();
            unsafe {
                highs_sys::Highs_getSolution(self.ptr, var_value_out.as_mut_ptr(), null, null, null)
            };
        }
    }

    pub fn get_dual_solution(&mut self, row_dual_out: &mut [f64]) {
        let num_rows = unsafe { highs_sys::Highs_getNumRow(self.ptr) } as usize;

        if !row_dual_out.is_empty() {
            // println!("Getting row dual");
            assert!(row_dual_out.len() == num_rows);

            let null = std::ptr::null_mut();
            unsafe {
                highs_sys::Highs_getSolution(self.ptr, null, null, null, row_dual_out.as_mut_ptr())
            };
        }
    }

    pub fn optimize(&mut self, var_value_out: &mut [f64], row_dual_out: &mut [f64]) -> Option<f64> {
        let _p = hprof::enter("lp optimize");

        // Remove the basis so we don't use the solver incrementally. It's better to use presolve.
        unsafe {highs_sys::Highs_setLogicalBasis(self.ptr) };

        let retval = unsafe { highs_sys::Highs_run(self.ptr) };
        drop(_p);
        let _p = hprof::enter("get_solution");
        let status = HighsStatus::try_from(retval);
        assert!(status == Ok(HighsStatus::OK));
        let model_status_retval = unsafe { highs_sys::Highs_getModelStatus(self.ptr) };
        let model_status = HighsModelStatus::try_from(model_status_retval);
        assert!(model_status == Ok(HighsModelStatus::Optimal) || model_status == Ok(HighsModelStatus::ModelEmpty));


        // println!("Solved {:?} {:?}", _status, _model_status);

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

        // println!(
        //     "primal feasible {} dual feasible {}",
        //     primal_solution_status == 2,
        //     dual_solution_status == 2
        // );
        // assert!(primal_solution_status == 2);
        // assert!(dual_solution_status == 2);

        if !var_value_out.is_empty() {
            assert!(var_value_out.len() == num_cols);

            let null = std::ptr::null_mut();
            unsafe {
                highs_sys::Highs_getSolution(self.ptr, var_value_out.as_mut_ptr(), null, null, null)
            };
        }

        if !row_dual_out.is_empty() {
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

        // println!("getsolution ok");

        Some(objective_value)
    }
}
