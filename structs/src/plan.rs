use serde::{Deserialize, Serialize};

use crate::{backend::Task, problem::Problem};

pub type Planner = Box<dyn FnMut(&Problem) -> Plan>;

#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct Plan {
    pub vehicle_tasks: Vec<Vec<PlanTask>>,
}

impl Plan {
    pub fn print(&self) {
        for (i, v) in self.vehicle_tasks.iter().enumerate() {
            println!("vehicle {}", i);
            for t in v {
                println!("  - {:?}", t);
            }
        }
    }

    pub fn update_pass_time(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        for v in self.vehicle_tasks.iter_mut() {
            assert!(!v.is_empty());
            if let Some(ext) = v[0].finish_cond.external.as_mut() {
                *ext -= dt;
            }

            for task in v.iter_mut() {
                if let Some(time) = task.finish_cond.time.as_mut() {
                    *time -= dt;

                    if *time <= 0.0 {
                        task.finish_cond.time = None;
                    }
                }
            }
        }
    }

    pub fn update_remove_ready_tasks(&mut self) {
        while let Some(v_ready) = self
            .vehicle_tasks
            .iter()
            .position(|v| v[0].finish_cond.is_ready())
        {
            for (v_idx, v) in self.vehicle_tasks.iter_mut().enumerate() {
                for task in v.iter_mut() {
                    if let Some((dep_v, dep_i)) = task.finish_cond.task_start {
                        assert!(dep_v != v_idx);
                        if dep_v == v_ready {
                            assert!(dep_i >= 1);
                            if dep_i == 1 {
                                task.finish_cond.task_start = None;
                            } else {
                                task.finish_cond.task_start = Some((dep_v, dep_i - 1));
                            }
                        }
                    }
                }
            }

            self.vehicle_tasks[v_ready].remove(0);
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct PlanTask {
    pub task: Task,
    pub finish_cond: Cond,
}

#[derive(Debug, Clone, Copy, Default)]
#[derive(Serialize, Deserialize)]
pub struct Cond {
    pub time: Option<f32>,
    pub external: Option<f32>,
    pub task_start: Option<(usize, usize)>,
}

impl Cond {
    pub fn empty() -> Self {
        Self::default()
    }
    pub fn event((v_idx, t_idx): (usize, usize)) -> Self {
        Self {
            task_start: Some((v_idx, t_idx)),
            ..Default::default()
        }
    }
    pub fn external(t: f32) -> Self {
        Self {
            external: Some(t),
            ..Default::default()
        }
    }
    pub fn time(t: f32) -> Self {
        Self {
            time: Some(t),
            ..Default::default()
        }
    }
}

impl Cond {
    pub fn is_ready(&self) -> bool {
        self.time.is_none() && self.external.is_none() && self.task_start.is_none()
    }
}
