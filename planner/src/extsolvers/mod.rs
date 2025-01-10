pub mod highs;
pub mod gurobi;

pub trait LPSolver {
    type Var : Copy+Clone;
    fn new() -> Self;
    fn add_var(&mut self, cost:f64 ) -> Self::Var;
    fn set_binary(&mut self, var :Self::Var);
    fn set_bounds(&mut self, var :Self::Var, lower :f64, upper :f64);
    fn add_constraint(&mut self, lb :f64, ub :f64, idxs :&[Self::Var], coeffs :&[f64]);
    fn set_time_limit(&mut self, seconds :f64);
    fn optimize(&mut self) -> Option<(f64, f64, Vec<f64>)>;
    fn inf(&self) -> f64;
    fn num_vars(&self) -> usize;
    fn write_model(&mut self) ;
}