use eframe::egui::{self, Align2, Color32, RichText};
use survsim_structs::{report::Report, Goal, TaskRef};

struct MyApp {
    report: Option<Report>,
    _mqtt_cli: paho_mqtt::Client,
    mqtt_rx: paho_mqtt::Receiver<Option<paho_mqtt::Message>>,
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(std::time::Duration::from_millis(25));
        while let Ok(Some(msg)) = self.mqtt_rx.try_recv() {
            let s = msg.payload_str();
            let report: Report = serde_json::from_slice(s.as_bytes()).unwrap();
            self.report = Some(report);
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("My egui Application");

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
                                    RichText::new("x").size(18.0),
                                )
                                .anchor(Align2::CENTER_BOTTOM),
                            );
                        }

                        for c in report.contacts.iter() {
                            plot_ui.text(
                                egui_plot::Text::new(
                                    [c.loc.x as f64, c.loc.y as f64].into(),
                                    RichText::new("c").size(18.0),
                                )
                                .color(if c.in_sight {
                                    Color32::RED
                                } else {
                                    Color32::GRAY
                                })
                                .anchor(Align2::CENTER_BOTTOM),
                            );
                        }

                        for d in report.drones.iter() {
                            plot_ui.text(
                                egui_plot::Text::new(
                                    [d.loc.x as f64, d.loc.y as f64].into(),
                                    RichText::new(&format!(
                                        "d ({}%)",
                                        (d.battery_level * 100.0).round() as i32
                                    ))
                                    .size(18.0),
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
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
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
    let mqtt_rx = mqtt_cli.start_consuming();

    eframe::run_native(
        "ATAM-MASIM: Current plan",
        options,
        Box::new(|cc| {
            let style = egui::Style {
                visuals: egui::Visuals::light(),
                ..egui::Style::default()
            };
            cc.egui_ctx.set_style(style);
            Ok(Box::new(MyApp {
                report: None,
                _mqtt_cli: mqtt_cli,
                mqtt_rx,
            }))
        }),
    )
}
