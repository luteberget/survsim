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
pub enum TaskRef {
    FixedTask(usize),
    Contact(usize),
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
pub enum Goal {
    TaskRef(TaskRef),
    Wait,
    Base,
}

#[derive(Serialize, Deserialize,Debug)]
pub struct FixedTaskReport {
    pub loc :Point,
}


#[derive(Serialize, Deserialize,Debug)]
pub struct ContactReport {
    pub loc :Point,
    pub in_sight :bool,
}

#[derive(Serialize, Deserialize,Debug)]
pub struct DroneReport {
    pub goal: Goal,
    pub loc: Point,
    pub base_dist: f32,
    pub fixed_task_dists: Vec<f32>,
    pub battery_level: f32,
    pub battery_consumption_traveling: f32,
    pub battery_consumption_hovering: f32,
    pub tasks_in_sight: Vec<TaskRef>,
}

#[derive(Clone, Serialize,Deserialize, Debug)]
pub struct FixedDists {
    pub fixed_task_base_dist: Vec<f32>,
    pub fixed_task_dists: Vec<Vec<f32>>,
}

#[derive(Serialize,Deserialize, Debug)]
pub struct Report {
    pub current_time: f32,
    pub fixed_dists: FixedDists,
    pub drones: Vec<DroneReport>,
    pub fixed_tasks :Vec<FixedTaskReport>,
    pub contacts :Vec<ContactReport>,
}
