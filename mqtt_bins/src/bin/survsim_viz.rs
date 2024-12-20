use eframe::{
    egui::{self, Align2, Color32, RichText},
    emath::OrderedFloat,
};
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

            let plan = &self.plan;
            let report = &self.report;


            survsim_viz::draw_map_and_plan(ui, plan, report);
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
