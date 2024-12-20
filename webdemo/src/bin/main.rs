#![warn(clippy::all, rust_2018_idioms)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use eframe::wasm_bindgen::JsValue;
use web_sys::{console, js_sys};

// When compiling natively:
#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result {
    env_logger::init(); // Log to stderr (if you run with `RUST_LOG=debug`).

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([400.0, 300.0])
            .with_min_inner_size([300.0, 220.0])
            .with_icon(
                // NOTE: Adding an icon is optional
                eframe::icon_data::from_png_bytes(&include_bytes!("../assets/icon-256.png")[..])
                    .expect("Failed to load icon"),
            ),
        ..Default::default()
    };
    eframe::run_native(
        "survsim web demo",
        native_options,
        Box::new(|cc| Ok(Box::new(survsim_webdemo::SurvSimApp::new(cc)))),
    )
}

// async fn complete_option_webtimeout<T>(mut f: impl FnMut() -> Option<T>) -> T {
//     loop {
//         if let Some(x) = f() {
//             return x;
//         }

//         gloo_timers::future::TimeoutFuture::new(0).await;
//     }
// }

// async fn compute() {
//     console::log_1(&JsValue::from_str("COMPUTE"));

//     struct Process {
//         step: usize,
//     }
//     impl Process {
//         pub fn next(&mut self) -> Option<u32> {
//             self.step += 1;
//             // console::log_1(&JsValue::from_f64(self.step as _));

//             (self.step >= 100_000_000).then_some(1)
//         }
//     }
//     let mut proc = Process { step: 0 };

//     let x = complete_option_webtimeout(|| proc.next()).await;
//     let x: JsValue = x.into();
//     console::log_1(&JsValue::from_str("hallo"));
//     console::log_1(&x);
// }

fn worker_new(name: &str) -> web_sys::Worker {
    let origin = web_sys::window()
        .expect("window to be available")
        .location()
        .origin()
        .expect("origin to be available");

    let script = js_sys::Array::new();
    script.push(
        &format!(r#"importScripts("{origin}/survsim/{name}.js");wasm_bindgen("{origin}/survsim/{name}_bg.wasm");"#)
            .into(),
    );

    let blob = web_sys::Blob::new_with_str_sequence_and_options(
        &script,
        web_sys::BlobPropertyBag::new().type_("text/javascript"),
    )
    .expect("blob creation succeeds");

    let url = web_sys::Url::create_object_url_with_blob(&blob).expect("url creation succeeds");

    web_sys::Worker::new(&url).expect("failed to spawn worker")
}

// When compiling to web using trunk:
#[cfg(target_arch = "wasm32")]
fn main() {
    use std::{cell::RefCell, rc::Rc};

    use eframe::wasm_bindgen::JsCast as _;
    use survsim_structs::{plan::Plan, report::Report, GoalMsg};
    use survsim_webdemo::WorkerResultMessage;

    // Redirect `log` message to `console.log` and friends:
    eframe::WebLogger::init(log::LevelFilter::Debug).ok();

    let web_options = eframe::WebOptions::default();

    let worker = worker_new("worker");

    let plan: Rc<RefCell<Option<Plan>>> = Rc::new(RefCell::new(None));
    let pending_msgs: Rc<RefCell<Vec<GoalMsg>>> = Rc::new(RefCell::new(Vec::new()));

    let plan_in_onmessage = plan.clone();
    let pending_msgs_in_onmessage = pending_msgs.clone();
    let onmessage =
        js_sys::wasm_bindgen::prelude::Closure::wrap(Box::new(move |msg: web_sys::MessageEvent| {
            let data_str = msg.data().as_string().unwrap();
            let message: WorkerResultMessage = serde_json::from_str(&data_str).unwrap();
            match message {
                WorkerResultMessage::Plan(plan) => {
                    web_sys::console::log_1(&"received plan".into());
                    *plan_in_onmessage.borrow_mut() = Some(plan);
                }
                WorkerResultMessage::Goal(goal_msg) => {
                    web_sys::console::log_1(&"received goal".into());
                    pending_msgs_in_onmessage.borrow_mut().push(goal_msg)
                }
            };
        })
            as Box<dyn Fn(web_sys::MessageEvent)>);
    worker.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
    onmessage.forget();

    let mk_report: Box<dyn FnMut(&Report)> = Box::new(move |r| {
        worker
            .post_message(&serde_json::to_string(&r).unwrap().into())
            .expect("posting result message succeeds");
    });

    wasm_bindgen_futures::spawn_local(async {
        let document = web_sys::window()
            .expect("No window")
            .document()
            .expect("No document");

        let canvas = document
            .get_element_by_id("the_canvas_id")
            .expect("Failed to find the_canvas_id")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("the_canvas_id was not a HtmlCanvasElement");

        let start_result = eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|cc| {
                    Ok(Box::new(survsim_webdemo::SurvSimApp::new(
                        cc,
                        plan,
                        pending_msgs,
                        mk_report,
                    )))
                }),
            )
            .await;

        // Remove the loading text and spinner:
        if let Some(loading_text) = document.get_element_by_id("loading_text") {
            match start_result {
                Ok(_) => {
                    loading_text.remove();
                }
                Err(e) => {
                    loading_text.set_inner_html(
                        "<p> The app has crashed. See the developer console for details. </p>",
                    );
                    panic!("Failed to start eframe: {e:?}");
                }
            }
        }
    });
}
