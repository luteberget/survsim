use serde::{Deserialize, Serialize};

use crate::{report::Location, TaskRef};

#[derive(Debug, Deserialize, Serialize)]
pub struct Problem {
    pub t_takeoff: f32,
    pub battery_capacity: f32,
    pub vehicles: Vec<Vehicle>,
    pub pois: Vec<Poi>,
    pub distances :Vec<((Location, Location), Dist)>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Vehicle {
    pub start_battery: f32,
    // pub start_airborne: bool,
    // pub start_time: f32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Poi {
    pub task_ref :TaskRef,
    pub reward_rate: f32,
    pub battery_rate: f32,
}

#[derive(Debug, Deserialize, Serialize)]
#[derive(Copy, Clone)]
pub struct Dist {
    pub dt: f32,
    pub d_batt: f32,
}
