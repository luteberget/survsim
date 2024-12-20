#![cfg(feature="highs")]

use std::ffi::{c_void, CStr};

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
        //         CStr::from_bytes_with_nul("pdlp\0".as_bytes())
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
                0,
            )
        };
        // unsafe {
        //     highs_sys::Highs_setIntOptionValue(
        //         ptr,
        //         CStr::from_bytes_with_nul("log_dev_level\0".as_bytes())
        //             .unwrap()
        //             .as_ptr(),
        //         2,
        //     )
        // };
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
        #[cfg(feature="prof")]
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
        #[cfg(feature="prof")]
        let _p = hprof::enter("lp optimize");

        let mut model_status = self.highs_solve();

        if model_status == Ok(HighsModelStatus::Unknown) {
            // Sometimes the model is dual infeasible. Maybe it's a bug in the LP solver? Let's just reduce dual tolerance to try to ignore it.
            println!("REDUCING DUAL FEASIBILITY TOLERANCE!");
            let ok = unsafe {
                highs_sys::Highs_setDoubleOptionValue(
                    self.ptr,
                    CStr::from_bytes_with_nul("dual_feasibility_tolerance\0".as_bytes())
                        .unwrap()
                        .as_ptr(),
                    0.02,
                )
            };
            assert!(ok == HighsStatuskOk);

            model_status = self.highs_solve();
        }

        assert!(
            model_status == Ok(HighsModelStatus::Optimal)
                || model_status == Ok(HighsModelStatus::ModelEmpty)
        );

        #[cfg(feature="prof")]
        drop(_p);
        #[cfg(feature="prof")]
        let _p = hprof::enter("get_solution");
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

    fn highs_solve(&mut self) -> Result<HighsModelStatus, InvalidStatus> {
        // Remove the basis so we don't use the solver incrementally. It's better to use presolve.
        unsafe { highs_sys::Highs_setLogicalBasis(self.ptr) };

        let _retval = unsafe { highs_sys::Highs_run(self.ptr) };
        // let status = HighsStatus::try_from(retval);
        // assert!(status == Ok(HighsStatus::OK));
        let model_status_retval = unsafe { highs_sys::Highs_getModelStatus(self.ptr) };
        HighsModelStatus::try_from(model_status_retval)
    }
}

use std::convert::TryFrom;
use std::fmt::{Debug, Formatter};
use std::num::TryFromIntError;
use std::os::raw::c_int;

use highs_sys::*;

/// The kinds of results of an optimization
#[derive(Clone, Copy, Debug, PartialOrd, PartialEq, Ord, Eq)]
pub enum HighsModelStatus {
    /// not initialized
    NotSet = MODEL_STATUS_NOTSET as isize,
    /// Unable to load model
    LoadError = MODEL_STATUS_LOAD_ERROR as isize,
    /// invalid model
    ModelError = MODEL_STATUS_MODEL_ERROR as isize,
    /// Unable to run the pre-solve phase
    PresolveError = MODEL_STATUS_PRESOLVE_ERROR as isize,
    /// Unable to solve
    SolveError = MODEL_STATUS_SOLVE_ERROR as isize,
    /// Unable to clean after solve
    PostsolveError = MODEL_STATUS_POSTSOLVE_ERROR as isize,
    /// No variables in the model: nothing to optimize
    ModelEmpty = MODEL_STATUS_MODEL_EMPTY as isize,
    /// There is no solution to the problem
    Infeasible = MODEL_STATUS_INFEASIBLE as isize,
    /// The problem in unbounded or infeasible
    UnboundedOrInfeasible = MODEL_STATUS_UNBOUNDED_OR_INFEASIBLE as isize,
    /// The problem is unbounded: there is no single optimal value
    Unbounded = MODEL_STATUS_UNBOUNDED as isize,
    /// An optimal solution was found
    Optimal = MODEL_STATUS_OPTIMAL as isize,
    /// objective bound
    ObjectiveBound = MODEL_STATUS_OBJECTIVE_BOUND as isize,
    /// objective target
    ObjectiveTarget = MODEL_STATUS_OBJECTIVE_TARGET as isize,
    /// reached limit
    ReachedTimeLimit = MODEL_STATUS_REACHED_TIME_LIMIT as isize,
    /// reached limit
    ReachedIterationLimit = MODEL_STATUS_REACHED_ITERATION_LIMIT as isize,
    /// Unknown model status
    Unknown = MODEL_STATUS_UNKNOWN as isize,
}

/// This error should never happen: an unexpected status was returned
#[derive(PartialEq, Clone, Copy)]
pub struct InvalidStatus(pub c_int);

impl Debug for InvalidStatus {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} is not a valid HiGHS model status. \
        This error comes from a bug in highs rust bindings. \
        Please report it.",
            self.0
        )
    }
}

impl TryFrom<c_int> for HighsModelStatus {
    type Error = InvalidStatus;

    fn try_from(value: c_int) -> Result<Self, Self::Error> {
        use highs_sys::*;
        match value {
            MODEL_STATUS_NOTSET => Ok(Self::NotSet),
            MODEL_STATUS_LOAD_ERROR => Ok(Self::LoadError),
            MODEL_STATUS_MODEL_ERROR => Ok(Self::ModelError),
            MODEL_STATUS_PRESOLVE_ERROR => Ok(Self::PresolveError),
            MODEL_STATUS_SOLVE_ERROR => Ok(Self::SolveError),
            MODEL_STATUS_POSTSOLVE_ERROR => Ok(Self::PostsolveError),
            MODEL_STATUS_MODEL_EMPTY => Ok(Self::ModelEmpty),
            MODEL_STATUS_INFEASIBLE => Ok(Self::Infeasible),
            MODEL_STATUS_UNBOUNDED => Ok(Self::Unbounded),
            MODEL_STATUS_UNBOUNDED_OR_INFEASIBLE => Ok(Self::UnboundedOrInfeasible),
            MODEL_STATUS_OPTIMAL => Ok(Self::Optimal),
            MODEL_STATUS_OBJECTIVE_BOUND => Ok(Self::ObjectiveBound),
            MODEL_STATUS_OBJECTIVE_TARGET => Ok(Self::ObjectiveTarget),
            MODEL_STATUS_REACHED_TIME_LIMIT => Ok(Self::ReachedTimeLimit),
            MODEL_STATUS_REACHED_ITERATION_LIMIT => Ok(Self::ReachedIterationLimit),
            MODEL_STATUS_UNKNOWN => Ok(Self::Unknown),
            n => Err(InvalidStatus(n)),
        }
    }
}

/// The status of a highs operation
#[derive(Clone, Copy, Debug, PartialOrd, PartialEq, Ord, Eq)]
pub enum HighsStatus {
    /// Success
    OK = 0,
    /// Done, with warning
    Warning = 1,
    /// An error occurred
    Error = 2,
}

impl From<TryFromIntError> for HighsStatus {
    fn from(_: TryFromIntError) -> Self {
        Self::Error
    }
}

impl TryFrom<c_int> for HighsStatus {
    type Error = InvalidStatus;

    fn try_from(value: c_int) -> Result<Self, InvalidStatus> {
        match value {
            STATUS_OK => Ok(Self::OK),
            STATUS_WARNING => Ok(Self::Warning),
            STATUS_ERROR => Ok(Self::Error),
            n => Err(InvalidStatus(n)),
        }
    }
}
