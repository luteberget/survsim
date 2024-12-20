
use backend::SurvsimBackend;
use executive::Executive;
use survsim_structs::plan::{Cond, Plan, PlanTask, Planner};
use survsim_structs::{backend::Task, problem::Problem, TaskRef};

pub mod backend;
pub mod executive;
pub mod parse_report;

fn main() {
    let mut planner: Planner = Box::new(survsim_planner::greedy::solve_greedy_cycles);
    let mut executive = Executive::new(&mut planner);
    SurvsimBackend::new().main_loop(|backend| executive.update(backend));
}

pub fn simple_planner(problem: &Problem) -> Plan {
    // Empty plan
    let mut plan = Plan {
        vehicle_tasks: problem
            .vehicles
            .iter()
            .map(|_| vec![PlanTask {task: Task::Wait, finish_cond: Cond::empty()}])
            .collect(),
    };

    let task = TaskRef::FixedTask(0);

    // Vehicle one go to task 1
    plan.vehicle_tasks[0].push(PlanTask {task: Task::Takeoff(task),finish_cond: Cond::external(30.0)});
    plan.vehicle_tasks[0].push(PlanTask {task: Task::GotoPoi(task),finish_cond: Cond::external(30.0)});
    plan.vehicle_tasks[0].push(PlanTask {task: Task::WatchPoi(task),finish_cond: Cond::event((1,3))});
    // Vehicle two go to task 1
    plan.vehicle_tasks[1][0].finish_cond.task_start = Some((0,3));
    plan.vehicle_tasks[1].push(PlanTask {task: Task::Takeoff(task),finish_cond: Cond::external(30.0)});
    plan.vehicle_tasks[1].push(PlanTask {task: Task::GotoPoi(task),finish_cond: Cond::external(30.0)});
    plan.vehicle_tasks[1].push(PlanTask {task: Task::WatchPoi(task),finish_cond: Cond::empty()});

    let mut waiting_for = (1usize,3usize);
    for _ in 0usize..10 {
        let v = (waiting_for.0 + 1)%2;
        plan.vehicle_tasks[v].push(PlanTask { task: Task::ApproachBase, finish_cond: Cond::external(30.0)});
        plan.vehicle_tasks[v].push(PlanTask { task: Task::Land, finish_cond: Cond::external(30.0)});
        plan.vehicle_tasks[v].push(PlanTask { task: Task::Takeoff(task), finish_cond: Cond::external(30.0)});
        plan.vehicle_tasks[v].push(PlanTask { task: Task::GotoPoi(task), finish_cond: Cond::external(30.0)});
        plan.vehicle_tasks[v].push(PlanTask { task: Task::WatchPoi(task), finish_cond: Cond::empty()});
        let last_task_idx = plan.vehicle_tasks[v].len() - 1;
        plan.vehicle_tasks[waiting_for.0][waiting_for.1].finish_cond.task_start = Some((v, last_task_idx));
        waiting_for = (v, last_task_idx);
    }

    for v in plan.vehicle_tasks.iter_mut() {
        v.push(PlanTask {task: Task::Wait, finish_cond: Cond::time(f32::INFINITY)});
    }

    plan.print();
    plan
}
