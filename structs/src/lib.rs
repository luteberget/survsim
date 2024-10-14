pub mod problem;
pub mod report;
pub mod backend;

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Serialize, Deserialize,Debug)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point {
    pub fn dist_xy(&self, other: &Point) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
    pub fn dist(&self, other: &Point) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn eq_xyz(&self, other: &Point) -> bool {
        self.dist(other) < 1e-3
    }
    pub fn eq_xy(&self, other: &Point) -> bool {
        self.dist_xy(other) < 1e-3
    }
}


#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
#[derive(PartialEq, Eq, PartialOrd, Ord)]
#[derive(Hash)]
pub enum TaskRef {
    FixedTask(usize),
    Contact(usize),
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
#[derive(PartialEq, Eq, PartialOrd, Ord)]
pub enum Goal {
    TaskRef(TaskRef),
    Wait,
    Base,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct GoalMsg {
    pub drone: usize,
    pub goal: Goal,
}