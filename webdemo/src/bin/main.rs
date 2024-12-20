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
        Box::new(|cc| Ok(Box::new(survsim_webdemo::TemplateApp::new(cc)))),
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
        &format!(r#"importScripts("{origin}/{name}.js");wasm_bindgen("{origin}/{name}_bg.wasm");"#)
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
    use eframe::wasm_bindgen::JsCast as _;

    // Redirect `log` message to `console.log` and friends:
    eframe::WebLogger::init(log::LevelFilter::Debug).ok();

    let web_options = eframe::WebOptions::default();

    let worker = worker_new("worker");
    let worker_clone = worker.clone();

    let onmessage = js_sys::wasm_bindgen::prelude::Closure::wrap(Box::new(move |msg: web_sys::MessageEvent| {
        let worker_clone = worker_clone.clone();
        let data = js_sys::Array::from(&msg.data());

        if data.length() == 0 {
            let msg = js_sys::Array::new();
            msg.push(&2.into());
            msg.push(&5.into());
            worker_clone
                .post_message(&msg.into())
                .expect("sending message to succeed");
        } else {
            let a = data
                .get(0)
                .as_f64()
                .expect("first array value to be a number") as u32;
            let b = data
                .get(1)
                .as_f64()
                .expect("second array value to be a number") as u32;
            let result = data
                .get(2)
                .as_f64()
                .expect("third array value to be a number") as u32;

            web_sys::console::log_1(&format!("{a} x {b} = {result}").into());
        }
    }) as Box<dyn Fn(web_sys::MessageEvent)>);
    worker.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
    onmessage.forget();


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
                Box::new(|cc| Ok(Box::new(survsim_webdemo::TemplateApp::new(cc)))),
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
