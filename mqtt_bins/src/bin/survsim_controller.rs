use survsim_controller::backend::SurvsimBackend;
use survsim_controller::executive::Executive;
use survsim_structs::plan::{Cond, Plan, PlanTask, Planner};
use survsim_structs::report::Report;
use survsim_structs::{backend, GoalMsg};
use survsim_structs::{backend::Task, problem::Problem, TaskRef};

fn main() {
    let (mut recv_fn, mut send_fn): (
        Box<dyn FnMut() -> Result<Report, ()>>,
        Box<dyn FnMut(GoalMsg)>,
    ) = {
        println!("connecting");
        let mqtt_opts = paho_mqtt::CreateOptionsBuilder::new()
            .server_uri("mqtt://localhost:1883")
            .finalize();
        let mqtt_cli = paho_mqtt::Client::new(mqtt_opts).unwrap();
        let conn_opts = paho_mqtt::ConnectOptionsBuilder::new()
            .keep_alive_interval(std::time::Duration::from_secs(20))
            .finalize();
        mqtt_cli.connect(conn_opts).unwrap();
        mqtt_cli.subscribe("/survsim/state", 1).unwrap();
        let mqtt_rx = mqtt_cli.start_consuming();

        let recv = Box::new(move || {
            let msg = match next_msg_skip_old(&mqtt_rx) {
                Some(x) => x,
                None => {
                    return Err(());
                }
            };

            let new_report =
                serde_json::from_slice::<Report>(msg.payload_str().as_bytes()).unwrap();
            Ok(new_report)
        });

        let send = Box::new(move |msg| {
            mqtt_cli
                .publish(paho_mqtt::Message::new(
                    "/survsim/goal",
                    serde_json::to_string(&msg).unwrap(),
                    1,
                ))
                .unwrap();
        });

        (recv, send)
    };

    let mut send_plan_fn: Box<dyn FnMut(&Plan)> = {
        let mqtt_opts = paho_mqtt::CreateOptionsBuilder::new()
            .server_uri("mqtt://localhost:1883")
            .finalize();
        let mqtt_cli = paho_mqtt::Client::new(mqtt_opts).unwrap();
        let conn_opts = paho_mqtt::ConnectOptionsBuilder::new()
            .keep_alive_interval(std::time::Duration::from_secs(20))
            .finalize();
        mqtt_cli.connect(conn_opts).unwrap();

        Box::new(move |plan| {
            mqtt_cli
                .publish(paho_mqtt::Message::new(
                    "/survsim/plan",
                    serde_json::to_string(plan).unwrap(),
                    1,
                ))
                .unwrap();
        })
    };

    let planner: Planner = Box::new(survsim_planner::greedy::solve_greedy_cycles);
    let mut executive = Executive::new(planner);
    let mut backend = SurvsimBackend::new();

    // Main loop
    while let Ok(new_report) = recv_fn() {
        backend.set_report(new_report);
        let new_plan = executive.update(&mut backend);
        send_plan_fn(&new_plan);
        backend.dispatch(&mut send_fn);
    }
}

fn next_msg_skip_old(
    c: &paho_mqtt::Receiver<Option<paho_mqtt::Message>>,
) -> Option<paho_mqtt::Message> {
    let mut msg = loop {
        match c.recv() {
            Ok(Some(x)) => break x,
            Ok(None) => {
                continue;
            }
            Err(_) => {
                return None;
            }
        }
    };

    // overwrite with later messages if there are some waiting
    while let Ok(Some(newer_msg)) = c.try_recv() {
        msg = newer_msg;
    }

    Some(msg)
}

pub fn simple_planner(problem: &Problem) -> Plan {
    // Empty plan
    let mut plan = Plan {
        vehicle_tasks: problem
            .vehicles
            .iter()
            .map(|_| {
                vec![PlanTask {
                    task: Task::Wait,
                    finish_cond: Cond::empty(),
                }]
            })
            .collect(),
    };

    let task = TaskRef::FixedTask(0);

    // Vehicle one go to task 1
    plan.vehicle_tasks[0].push(PlanTask {
        task: Task::Takeoff(task),
        finish_cond: Cond::external(30.0),
    });
    plan.vehicle_tasks[0].push(PlanTask {
        task: Task::GotoPoi(task),
        finish_cond: Cond::external(30.0),
    });
    plan.vehicle_tasks[0].push(PlanTask {
        task: Task::WatchPoi(task),
        finish_cond: Cond::event((1, 3)),
    });
    // Vehicle two go to task 1
    plan.vehicle_tasks[1][0].finish_cond.task_start = Some((0, 3));
    plan.vehicle_tasks[1].push(PlanTask {
        task: Task::Takeoff(task),
        finish_cond: Cond::external(30.0),
    });
    plan.vehicle_tasks[1].push(PlanTask {
        task: Task::GotoPoi(task),
        finish_cond: Cond::external(30.0),
    });
    plan.vehicle_tasks[1].push(PlanTask {
        task: Task::WatchPoi(task),
        finish_cond: Cond::empty(),
    });

    let mut waiting_for = (1usize, 3usize);
    for _ in 0usize..10 {
        let v = (waiting_for.0 + 1) % 2;
        plan.vehicle_tasks[v].push(PlanTask {
            task: Task::ApproachBase,
            finish_cond: Cond::external(30.0),
        });
        plan.vehicle_tasks[v].push(PlanTask {
            task: Task::Land,
            finish_cond: Cond::external(30.0),
        });
        plan.vehicle_tasks[v].push(PlanTask {
            task: Task::Takeoff(task),
            finish_cond: Cond::external(30.0),
        });
        plan.vehicle_tasks[v].push(PlanTask {
            task: Task::GotoPoi(task),
            finish_cond: Cond::external(30.0),
        });
        plan.vehicle_tasks[v].push(PlanTask {
            task: Task::WatchPoi(task),
            finish_cond: Cond::empty(),
        });
        let last_task_idx = plan.vehicle_tasks[v].len() - 1;
        plan.vehicle_tasks[waiting_for.0][waiting_for.1]
            .finish_cond
            .task_start = Some((v, last_task_idx));
        waiting_for = (v, last_task_idx);
    }

    for v in plan.vehicle_tasks.iter_mut() {
        v.push(PlanTask {
            task: Task::Wait,
            finish_cond: Cond::time(f32::INFINITY),
        });
    }

    plan.print();
    plan
}
