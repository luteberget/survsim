use crate::{problem::Problem, TaskRef};

#[derive(Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BackendTaskRef(pub usize);

#[derive(Debug)]
pub struct DroneTask {
    pub vehicle: usize,
    pub task: Task,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Task {
    Wait,
    Takeoff(TaskRef),
    ApproachBase,
    Land,
    GotoPoi(TaskRef),
    WatchPoi(TaskRef),
}

impl Task {
    pub fn eq_ignore_takeoff_direction(&self, other: &Task) -> bool {
        if matches!(self, Task::Takeoff(_)) && matches!(other, Task::Takeoff(_)) {
            return true;
        }
        self == other
    }
}

pub enum BackendTaskEvent {
    Completed(BackendTaskRef),
}

pub trait Backend {
    fn start_task(&mut self, task: DroneTask) -> BackendTaskRef;
    fn end_task(&mut self, task_ref: BackendTaskRef) -> bool;
    fn state(&self) -> (f32, &Problem);
    fn next_event(&mut self) -> Option<BackendTaskEvent>;
}
