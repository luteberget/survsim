#![warn(clippy::all, rust_2018_idioms)]

mod app;
pub use app::SurvSimApp;
use survsim_structs::{plan::Plan, GoalMsg};

#[derive(serde::Serialize, serde::Deserialize)]
pub enum WorkerResultMessage {
    Plan(Plan),
    Goal(GoalMsg)
}