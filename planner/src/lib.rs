pub mod colgen;
pub mod gbfs;
pub mod shortest_path;
pub mod txgraph;
pub mod lpsolver;
pub mod highs_status;

pub fn round_time(t: f32, scale: i32) -> i32 {
    (t / scale as f32).round() as i32 * scale
}

pub fn ceil_time(t: f32, scale: i32) -> i32 {
    ((t-0.1) / scale as f32 ).ceil() as i32 * scale
}

#[derive(Debug)]
pub struct VehicleSolution {
    pub cost: f32,
    pub cost_including_shadow_price: f32,
    pub path: Vec<u32>,
}

