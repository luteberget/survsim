use survsim_structs::{backend::Task, problem::Problem};

pub type Planner = Box<dyn FnMut(&Problem) -> Plan>;

#[derive(Debug)]
pub struct Plan {
    pub vehicle_tasks: Vec<Vec<PlanTask>>,
}

impl Plan {
    pub fn update_pass_time(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        for v in self.vehicle_tasks.iter_mut() {
            assert!(!v.is_empty());
            if let Some(ext) = v[0].dependencies.external.as_mut() {
                *ext -= dt;
            }

            for task in v.iter_mut() {
                if let Some(time) = task.dependencies.time.as_mut() {
                    *time -= dt;

                    if *time <= 0.0 {
                        task.dependencies.time = None;
                    }
                }
            }
        }
    }

    pub fn update_remove_ready_tasks(&mut self) {
        while let Some(v_ready) = self
            .vehicle_tasks
            .iter()
            .position(|v| v[0].dependencies.is_ready())
        {
            for (v_idx, v) in self.vehicle_tasks.iter_mut().enumerate() {
                for task in v.iter_mut() {
                    if let Some((dep_v, dep_i)) = task.dependencies.event_started {
                        assert!(dep_v != v_idx);
                        if dep_v == v_ready {
                            assert!(dep_i >= 1);
                            if dep_i == 1 {
                                task.dependencies.event_started = None;
                            } else {
                                task.dependencies.event_started = Some((dep_v, dep_i - 1));
                            }
                        }
                    }
                }
            }

            self.vehicle_tasks[v_ready].remove(0);
        }
    }
}

#[derive(Debug)]
#[derive(Clone, Copy)]
pub struct PlanTask {
    pub task: Task,
    pub dependencies: Dependencies,
}

#[derive(Debug)]
#[derive(Clone, Copy)]
#[derive(Default)]
pub struct Dependencies {
    pub time: Option<f32>,
    pub external: Option<f32>,
    pub event_started: Option<(usize, usize)>,
}

impl Dependencies {
    pub fn empty() -> Self {
        Self::default()
    }
    pub fn event((v_idx,t_idx) :(usize,usize)) -> Self {
        Self {
            event_started: Some((v_idx,t_idx)), ..Default::default()
        }
    }
    pub fn external(t :f32) -> Self {
        Self {
            external: Some(t), ..Default::default()
        }
    }
    pub fn wait(t :f32) -> Self {
        Self {
            time: Some(t), ..Default::default()
        }
    }
}

impl Dependencies {
    pub fn is_ready(&self) -> bool {
        self.time.is_none() && self.external.is_none() && self.event_started.is_none()
    }
}
