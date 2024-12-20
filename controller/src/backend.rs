use std::collections::VecDeque;

use survsim_structs::{
    backend::{Backend, BackendTaskEvent, BackendTaskRef, DroneTask, Task},
    problem::Problem,
    report::Report,
    Goal, GoalMsg,
};

use crate::parse_report::create_planning_problem;

struct RunningTask {
    task_ref: BackendTaskRef,
    task: Task,
    start_time: f32,
}

pub struct SurvsimBackend {
    pending_events: VecDeque<BackendTaskEvent>,
    current_problem: Option<Problem>,
    current_report: Option<Report>,
    prev_drone_goals: Vec<Goal>,
    curr_drone_task: Vec<Option<RunningTask>>,
    task_id_counter: usize,
    current_time: f32,
}

impl SurvsimBackend {
    pub fn new() -> Self {
        Self {
            curr_drone_task: Default::default(),
            current_problem: None,
            current_report: None,
            current_time: 0.0,
            pending_events: Default::default(),
            prev_drone_goals: Default::default(),
            task_id_counter: 0,
        }
    }

    pub fn set_report(
        &mut self,
        new_report: Report,
        
    ) {
        self.current_time = new_report.current_time;

        while new_report.drones.len() >= self.curr_drone_task.len() {
            self.curr_drone_task.push(None);
        }

        self.current_problem = Some(create_planning_problem(&new_report));
        self.current_report = Some(new_report);
        self.check_task_finished_external();
    }

    pub fn dispatch(&mut self, mut f :impl FnMut(GoalMsg)) {

        // Diff and dispatch.
        for v_idx in 0..self.curr_drone_task.len() {
            while v_idx >= self.prev_drone_goals.len() {
                self.prev_drone_goals.push(Goal::Wait);
            }

            let next_goal: Goal = match self.curr_drone_task[v_idx]
                .as_ref()
                .map(|x| x.task)
                .unwrap_or(Task::Wait)
            {
                Task::Wait => Goal::Wait,
                Task::Takeoff(task_ref) | Task::GotoPoi(task_ref) | Task::WatchPoi(task_ref) => {
                    Goal::TaskRef(task_ref)
                }
                Task::ApproachBase | Task::Land => Goal::Base,
            };

            if self.prev_drone_goals[v_idx] != next_goal {
                let msg = GoalMsg {
                    drone: v_idx,
                    goal: next_goal,
                };
                println!("t={:.2} Dispatching {:?}", self.current_time, msg);
                f(msg);

                self.prev_drone_goals[v_idx] = next_goal;
            }
        }
    }

    fn check_task_finished_external(&mut self) {
        let report = match &self.current_report {
            Some(r) => r,
            None => return,
        };

        for (v_idx, option_running_task) in self.curr_drone_task.iter_mut().enumerate() {
            if let Some(running_task) = option_running_task.as_mut() {
                let is_finished_external = match running_task.task {
                    Task::ApproachBase => report.drones[v_idx].at_base,
                    Task::Land => !report.drones[v_idx].is_airborne,
                    Task::Takeoff(_) => {
                        report.current_time - running_task.start_time
                            >= report.takeoff_separation_time
                    }
                    Task::GotoPoi(t) => report.drones[v_idx].tasks_in_sight.contains(&t),
                    Task::WatchPoi(_) | Task::Wait => false,
                };

                if is_finished_external {
                    self.pending_events
                        .push_back(BackendTaskEvent::Completed(running_task.task_ref));
                    *option_running_task = None;
                }
            }
        }
    }

    fn vehicle_from_taskref(&self, task_ref: BackendTaskRef) -> Option<usize> {
        self.curr_drone_task.iter().enumerate().find_map(|(d, rt)| {
            (rt.as_ref().map(|rt| rt.task_ref.0) == Some(task_ref.0)).then_some(d)
        })
    }
}

impl Default for SurvsimBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl Backend for SurvsimBackend {
    fn start_task(&mut self, task: DroneTask) -> BackendTaskRef {
        if self.curr_drone_task[task.vehicle].is_some() {
            panic!("vehicle already busy in start_task({:?})", task);
        }
        // TODO do we need to check if we are in a correct state to start the task?

        let task_id = self.task_id_counter;
        self.task_id_counter += 1;
        self.curr_drone_task[task.vehicle] = Some(RunningTask {
            task_ref: BackendTaskRef(task_id),
            task: task.task,
            start_time: self.current_time,
        });
        self.check_task_finished_external();
        BackendTaskRef(task_id)
    }

    fn end_task(&mut self, task_ref: BackendTaskRef) -> bool {
        let v_idx = match self.vehicle_from_taskref(task_ref) {
            Some(v_idx) => v_idx,
            None => return false,
        };

        // TODO: the ATAM MASIM needs to finish all takeoffs.

        // let task = match self.curr_drone_task[v_idx].as_ref() {
        //     Some(t) => &t.task,
        //     None => return false,
        // };

        // if matches!(task, Task::Takeoff(_) | Task::Land) {
        //     panic!("cannot cancel takeoff/landing");
        // }

        self.curr_drone_task[v_idx] = None;
        true
    }

    fn state(&self) -> (f32, &Problem) {
        (self.current_time, self.current_problem.as_ref().unwrap())
    }

    fn next_event(&mut self) -> Option<BackendTaskEvent> {
        self.pending_events.pop_front()
    }
}
