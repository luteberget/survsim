use eframe::{
    egui::{self, Align2, Color32, RichText},
    emath::OrderedFloat,
};
use egui_plot::Polygon;
use survsim_structs::{backend::Task, plan::Plan, report::Report, Goal, TaskRef};

struct MyApp {
    report: Option<Report>,
    plan: Option<Plan>,
    mqtt_cli: paho_mqtt::Client,
    mqtt_rx: paho_mqtt::Receiver<Option<paho_mqtt::Message>>,
    time_factor: f32,
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.set_zoom_factor(2.0);
        ctx.request_repaint_after(std::time::Duration::from_millis(25));
        while let Ok(Some(msg)) = self.mqtt_rx.try_recv() {
            if msg.topic() == "/survsim/state" {
                let s = msg.payload_str();
                let report: Report = serde_json::from_slice(s.as_bytes()).unwrap();
                self.report = Some(report);
            } else if msg.topic() == "/survsim/plan" {
                let s = msg.payload_str();
                let plan: Plan = serde_json::from_slice(s.as_bytes()).unwrap();
                self.plan = Some(plan);
            } else {
                panic!("unexpected message topic {}", msg.topic());
            }
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::TopBottomPanel::top("top").show_inside(ui, |ui| {
                let response = ui.add(
                    egui::Slider::new(&mut self.time_factor, 0.1..=60.0)
                        .text("simulation time factor"),
                );
                if response.changed() {
                    self.mqtt_cli
                        .publish(paho_mqtt::Message::new(
                            "/survsim/time_factor",
                            serde_json::to_string(&self.time_factor).unwrap(),
                            1,
                        ))
                        .unwrap();
                }
            });
            egui::TopBottomPanel::bottom("bottom").show_inside(ui, |ui| {
                if let Some(plan) = &self.plan {
                    draw_plan(ui, plan, "plan");
                }
            });

            egui_plot::Plot::new("map")
                .allow_drag(true)
                .allow_zoom(true)
                .show(ui, |plot_ui| {
                    // Draw fixed tasks
                    if let Some(report) = &self.report {
                        for ft in report.fixed_tasks.iter() {
                            plot_ui.text(
                                egui_plot::Text::new(
                                    [ft.loc.x as f64, ft.loc.y as f64].into(),
                                    RichText::new("x").size(22.0),
                                )
                                .anchor(Align2::CENTER_BOTTOM),
                            );
                        }

                        for c in report.contacts.iter() {
                            plot_ui.text(
                                egui_plot::Text::new(
                                    [c.loc.x as f64, c.loc.y as f64].into(),
                                    RichText::new("c").size(22.0),
                                )
                                .color(if c.in_sight {
                                    Color32::RED
                                } else {
                                    Color32::GRAY
                                })
                                .anchor(Align2::CENTER_BOTTOM),
                            );
                        }

                        for (di, d) in report.drones.iter().enumerate() {
                            const DRONE_DRAW_OFFSET: f64 = 5.0;
                            plot_ui.text(
                                egui_plot::Text::new(
                                    [d.loc.x as f64, d.loc.y as f64].into(),
                                    RichText::new("õ").size(22.0),
                                )
                                .color(if d.is_airborne {
                                    Color32::DARK_BLUE
                                } else {
                                    Color32::BLUE
                                })
                                .anchor(Align2::CENTER_TOP),
                            );
                            plot_ui.text(
                                egui_plot::Text::new(
                                    [
                                        d.loc.x as f64 + di as f64 * DRONE_DRAW_OFFSET,
                                        d.loc.y as f64 - di as f64 * DRONE_DRAW_OFFSET,
                                    ]
                                    .into(),
                                    RichText::new(&format!(
                                        "\nd{} ({}%)",
                                        di + 1,
                                        (d.battery_level * 100.0).round() as i32
                                    ))
                                    .size(16.0),
                                )
                                .color(Color32::BLUE)
                                .anchor(Align2::CENTER_TOP),
                            );

                            let mut goal_task: Option<TaskRef> = None;

                            match d.goal {
                                Goal::TaskRef(task_ref) => {
                                    goal_task = Some(task_ref);
                                    let is_sighted = d.tasks_in_sight.contains(&task_ref);
                                    let target_pt = match task_ref {
                                        TaskRef::FixedTask(i) => report.fixed_tasks[i].loc,
                                        TaskRef::Contact(i) => report.contacts[i].loc,
                                    };
                                    plot_ui.arrows(
                                        egui_plot::Arrows::new(
                                            egui_plot::PlotPoints::from(vec![[
                                                d.loc.x as f64,
                                                d.loc.y as f64,
                                            ]]),
                                            egui_plot::PlotPoints::from(vec![[
                                                target_pt.x as f64,
                                                target_pt.y as f64,
                                            ]]),
                                        )
                                        .tip_length(10.0)
                                        .color(if is_sighted {
                                            Color32::DARK_BLUE
                                        } else {
                                            Color32::BLACK
                                        })
                                        .highlight(!is_sighted),
                                    );
                                }
                                Goal::Wait => {
                                    if d.is_airborne {
                                        plot_ui.text(
                                            egui_plot::Text::new(
                                                [d.loc.x as f64, d.loc.y as f64].into(),
                                                RichText::new("?").size(25.0),
                                            )
                                            .color(Color32::YELLOW)
                                            .anchor(Align2::LEFT_BOTTOM),
                                        )
                                    }
                                }
                                Goal::Base => {
                                    if !d.at_base {
                                        plot_ui.arrows(
                                            egui_plot::Arrows::new(
                                                egui_plot::PlotPoints::from(vec![[
                                                    d.loc.x as f64,
                                                    d.loc.y as f64,
                                                ]]),
                                                egui_plot::PlotPoints::from(vec![[
                                                    d.base.x as f64,
                                                    d.base.y as f64,
                                                ]]),
                                            )
                                            .tip_length(10.0),
                                        );
                                    }
                                }
                            }

                            for (i, ft) in report.fixed_tasks.iter().enumerate() {
                                let t = TaskRef::FixedTask(i);
                                if Some(t) != goal_task && d.tasks_in_sight.contains(&t) {
                                    plot_ui.line(
                                        egui_plot::Line::new(vec![
                                            [d.loc.x as f64, d.loc.y as f64],
                                            [ft.loc.x as f64, ft.loc.y as f64],
                                        ])
                                        .color(Color32::LIGHT_BLUE),
                                    );
                                }
                            }

                            for (i, c) in report.contacts.iter().enumerate() {
                                let t = TaskRef::Contact(i);
                                if Some(t) != goal_task && d.tasks_in_sight.contains(&t) {
                                    plot_ui.line(
                                        egui_plot::Line::new(vec![
                                            [d.loc.x as f64, d.loc.y as f64],
                                            [c.loc.x as f64, c.loc.y as f64],
                                        ])
                                        .color(Color32::LIGHT_BLUE),
                                    );
                                }
                            }
                        }
                    }
                });
        });
    }
}

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        // viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
        ..Default::default()
    };

    let mqtt_opts = paho_mqtt::CreateOptionsBuilder::new()
        .server_uri("mqtt://localhost:1883")
        .finalize();
    let mqtt_cli = paho_mqtt::Client::new(mqtt_opts).unwrap();
    let conn_opts = paho_mqtt::ConnectOptionsBuilder::new()
        .keep_alive_interval(std::time::Duration::from_secs(20))
        .finalize();
    mqtt_cli.connect(conn_opts).unwrap();
    mqtt_cli.subscribe("/survsim/state", 1).unwrap();
    mqtt_cli.subscribe("/survsim/plan", 1).unwrap();
    let mqtt_rx = mqtt_cli.start_consuming();

    eframe::run_native(
        "ATAM-survsim",
        options,
        Box::new(|cc| {
            let style = egui::Style {
                visuals: egui::Visuals::light(),
                ..egui::Style::default()
            };
            cc.egui_ctx.set_style(style);
            Ok(Box::new(MyApp {
                report: None,
                plan: None,
                mqtt_cli,
                mqtt_rx,
                time_factor: 1.0,
            }))
        }),
    )
}

fn get_plan_timing(plan: &Plan) -> Vec<Vec<(f32, f32)>> {
    let mut vtimes: Vec<Vec<(f32, f32)>> = plan.vehicle_tasks.iter().map(|_| Vec::new()).collect();
    let mut curr_idx = plan
        .vehicle_tasks
        .iter()
        .map(|_| (0, 0.0f32))
        .collect::<Vec<(usize, f32)>>();

    loop {
        let ready_task = curr_idx
            .iter()
            .enumerate()
            .filter(|(v, (t, _))| *t + 1 < plan.vehicle_tasks[*v].len())
            .find(|(v, (t, _))| {
                // The task is ready if it doesn't depend on another task.
                plan.vehicle_tasks[*v][*t]
                    .finish_cond
                    .task_start
                    // Skip if the task has already been processed
                    .filter(|(v2, t2)| curr_idx[*v2].0 < *t2)
                    .is_none()
            });

        // Find a vehicle that is ready to proceed.
        if let Some((v, (t, start))) = ready_task {
            let deps = plan.vehicle_tasks[v][*t].finish_cond;

            let end_time = deps
                .time
                .iter()
                .copied()
                .chain(deps.external.iter().map(|t| *start + *t))
                .chain(deps.task_start.iter().map(|(v2, t2)| {
                    if curr_idx[*v2].0 == *t2 {
                        curr_idx[*v2].1
                    } else {
                        vtimes[*v2][*t2].0
                    }
                }))
                .map(OrderedFloat)
                .max()
                .map(|x| x.0)
                .unwrap_or(0.0)
                .max(10.0);

            vtimes[v].push((*start, end_time));
            curr_idx[v] = (*t + 1, end_time);
        } else {
            // Did we reach the last task on all vehciels?
            let all_done = curr_idx
                .iter()
                .enumerate()
                .all(|(v, (t, _))| *t + 1 >= plan.vehicle_tasks[v].len());
            assert!(all_done);
            break;
        }
    }

    vtimes
}

fn draw_plan(ui: &mut egui::Ui, plan: &Plan, id_source: impl std::hash::Hash) {
    let timing = get_plan_timing(plan);
    egui_plot::Plot::new(id_source)
        .allow_drag(true)
        .allow_zoom(true)
        .height(400.0)
        // .view_aspect(1.)
        .show(ui, |plot_ui| {
            for (v_idx, v_plan) in plan.vehicle_tasks.iter().enumerate() {
                for (t_idx, plan_task) in v_plan.iter().enumerate().take(v_plan.len() - 1) {
                    let y0 = v_idx as f64;
                    let y1 = y0 + 1.0;
                    let (t0, t1) = timing[v_idx][t_idx];
                    let t0 = t0 as f64;
                    let t1 = t1 as f64;
                    let color = match plan_task.task {
                        Task::Wait => Color32::GRAY,
                        Task::Takeoff(_) => Color32::RED,
                        Task::ApproachBase => Color32::YELLOW,
                        Task::Land => Color32::RED,
                        Task::GotoPoi(_) => Color32::BLUE,
                        Task::WatchPoi(_) => Color32::DARK_GREEN,
                    };

                    if let Task::WatchPoi(task_ref) = plan_task.task {
                        plot_ui.text(
                            egui_plot::Text::new(
                                [0.5 * (t0 + t1), 0.5 * (y0 + y1)].into(),
                                RichText::new(format!("{:?}", task_ref)).size(14.0),
                            )
                            .anchor(Align2::CENTER_CENTER),
                        );
                    }

                    plot_ui.polygon(
                        Polygon::new(vec![[t0, y0], [t1, y0], [t1, y1], [t0, y1]])
                            .stroke(egui::Stroke::new(1.0, color)),
                    );

                    // Does it need an arrow
                    if let Some((v2, s2)) = plan_task.finish_cond.task_start {
                        let (a, b) = timing[v2][s2];
                        let a = a as f64;
                        let b = b as f64;

                        plot_ui.arrows(
                            egui_plot::Arrows::new(
                                egui_plot::PlotPoints::from(vec![[0.9 * t1 + 0.1 * t0, y0 + 0.5]]),
                                egui_plot::PlotPoints::from(vec![[
                                    0.9 * a + 0.1 * b,
                                    v2 as f64 + 0.5,
                                ]]),
                            )
                            .tip_length(10.0)
                            .color(Color32::GOLD),
                        );
                    }
                }
            }

            // let vehicles = seq
            //     .tasks
            //     .iter()
            //     .map(|t| t.vehicle_idx)
            //     .collect::<std::collections::HashSet<_>>();
            // plot_ui.line(
            //     egui_plot::Line::new(vec![[*time, 0.0], [*time, vehicles.len() as f64]])
            //         .color(Color32::BLUE),
            // );
            // for vehicle_idx in vehicles {
            //     // plot_ui.text(egui_plot::Text::new(value::new()));
            //     plot_ui.text(
            //         egui_plot::Text::new(
            //             [0.0, vehicle_idx as f64 + 0.5].into(),
            //             RichText::new(format!(
            //                 "{}",
            //                 vehicle_names
            //                     .get(vehicle_idx)
            //                     .map(|x| x.as_str())
            //                     .unwrap_or("unnamed vehicle")
            //             ))
            //             .size(14.0),
            //         )
            //         .anchor(Align2::RIGHT_CENTER),
            //     );
            // }
        });
}
