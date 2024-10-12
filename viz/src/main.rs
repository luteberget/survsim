use eframe::egui::{self, Align2, Color32, RichText};
use survsim_structs::Report;

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
                                    RichText::new(&format!("d ({}%)", (d.battery_level * 100.0).round() as i32)).size(18.0),
                                )
                                .color(Color32::BLUE)
                                .anchor(Align2::CENTER_TOP),
                            );
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
