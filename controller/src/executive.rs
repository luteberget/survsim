use core::f32;
use std::collections::HashMap;

use survsim_structs::plan::{Plan, PlanTask, Planner};
use survsim_structs::{
    backend::{Backend, BackendTaskRef, DroneTask},
    TaskRef,
};

struct ExecutionState {
    time: f32,
    plan_time: f32,
    plan: Plan,
    #[allow(dead_code)]
    planned_tasks: Vec<TaskRef>,
    // last_print: f32,
}

#[derive(Debug)]
struct BackendTask {
    v_idx: usize,
    plan_task: PlanTask,
    finished: bool,
}

pub struct Executive<'a> {
    planner: &'a mut Planner,
    state: Option<ExecutionState>,
    backend_tasks: HashMap<BackendTaskRef, BackendTask>,
    vehicle_current_task: Vec<Option<BackendTaskRef>>,
}

impl<'a> Executive<'a> {
    pub fn update(&mut self, backend: &mut dyn Backend) {
        let (current_time, problem) = backend.state();

        std::fs::write("tiny_problem.json", serde_json::to_string(&problem).unwrap()).unwrap();
        println!("SAVING PROBLEM");

        let mut current_tasks = problem.pois.iter().map(|p| p.task_ref).collect::<Vec<_>>();
        current_tasks.sort();
        while problem.vehicles.len() >= self.vehicle_current_task.len() {
            self.vehicle_current_task.push(None);
        }

        let dt = self
            .state
            .as_ref()
            .map(|s| current_time - s.time)
            .unwrap_or(0.0);

        let replan = if let Some(_state) = self.state.as_ref() {
            false //current_time - state.plan_time >= 20.0e9 || state.planned_tasks != current_tasks
        } else {
            true
        };

        // let mut last_print = self
        //     .state
        //     .as_ref()
        //     .map(|x| x.last_print)
        //     .unwrap_or(f32::NEG_INFINITY);

        let (plan_time, mut plan) = if replan {
            let new_plan = (*self.planner)(problem);
            (current_time, new_plan)
        } else {
            self.state.take().map(|s| (s.plan_time, s.plan)).unwrap()
        };

        plan.update_pass_time(dt);

        'dispatch: loop {
            // Process external events
            while let Some(survsim_structs::backend::BackendTaskEvent::Completed(t)) =
                backend.next_event()
            {
                let backend_task = self.backend_tasks.get_mut(&t).unwrap();
                assert!(!backend_task.finished);
                backend_task.finished = true;

                let vehicle_plan = &mut plan.vehicle_tasks[backend_task.v_idx];
                let is_correct_task = !vehicle_plan.is_empty()
                    && vehicle_plan[0]
                        .task
                        .eq_ignore_takeoff_direction(&backend_task.plan_task.task);
                if !is_correct_task {
                    println!(
                        "WARNING: backend reported unexpected task finished {:?}",
                        backend_task
                    );
                } else {
                    println!("External task finished {:?}", backend_task);
                    vehicle_plan[0].dependencies.external = None;
                }
            }

            plan.update_remove_ready_tasks();

            // Diff and dispatch
            for (v_idx, plan_task) in plan.vehicle_tasks.iter().map(|v| &v[0]).enumerate() {
                let start_new_task = if let Some(old_task_ref) = self.vehicle_current_task[v_idx] {
                    let backend_task = &self.backend_tasks[&old_task_ref];
                    if !backend_task
                        .plan_task
                        .task
                        .eq_ignore_takeoff_direction(&plan_task.task)
                    {
                        if !backend_task.finished {
                            backend.end_task(old_task_ref);
                        }
                        self.backend_tasks.remove(&old_task_ref);
                        true
                    } else {
                        // If the current task is the same, no need to start a new one.
                        false
                    }
                } else {
                    true
                };

                if start_new_task {
                    println!("launching v={:?} {:?}",v_idx, plan_task.task);
                    let new_task_ref = backend.start_task(DroneTask {
                        vehicle: v_idx,
                        task: plan_task.task,
                    });
                    self.vehicle_current_task[v_idx] = Some(new_task_ref);
                    self.backend_tasks.insert(
                        new_task_ref,
                        BackendTask {
                            v_idx,
                            plan_task: *plan_task,
                            finished: false,
                        },
                    );
                    continue 'dispatch;
                }
            }

            break 'dispatch;
        }

        self.state = Some(ExecutionState {
            time: current_time,
            plan_time,
            plan,
            planned_tasks: current_tasks,
            // last_print,
        })
    }

    pub fn new(planner: &'a mut Planner) -> Self {
        Self {
            planner,
            backend_tasks: Default::default(),
            vehicle_current_task: Default::default(),
            state: None,
        }
    }
}
