use serde::{Deserialize, Serialize};

use crate::{Goal, Point, TaskRef};

#[derive(Serialize, Deserialize, Debug, Hash, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum Location {
    Base,
    DroneInitial(usize),
    Task(TaskRef),
    SinkNode,
}

impl Location {
    pub fn poi(&self) -> Option<TaskRef> {
        match self {
            Location::Task(task_ref) => Some(*task_ref),
            _ => None,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct DistanceList {
    pub entries: Vec<(Location, Location, f32)>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct FixedTaskReport {
    pub loc: Point,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ContactReport {
    pub loc: Point,
    pub in_sight: bool,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct DroneReport {
    pub goal: Goal,
    pub loc: Point,
    pub base: Point,
    pub battery_level: f32,
    pub battery_consumption_traveling: f32,
    pub battery_consumption_hovering: f32,
    pub tasks_in_sight: Vec<TaskRef>,
    pub at_base: bool,
    pub is_airborne: bool,
}

// #[derive(Clone, Serialize,Deserialize, Debug)]
// pub struct FixedDists {
//     pub fixed_task_base_dist: Vec<f32>,
//     pub fixed_task_dists: Vec<Vec<f32>>,
// }

#[derive(Serialize, Deserialize, Debug)]
pub struct Report {
    pub current_time: f32,
    pub takeoff_separation_time: f32,

    pub fixed_tasks: Vec<FixedTaskReport>,
    pub drones: Vec<DroneReport>,
    pub contacts: Vec<ContactReport>,
    pub distances: DistanceList,
}
