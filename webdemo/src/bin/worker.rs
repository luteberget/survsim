use std::{cell::RefCell, rc::Rc};

use eframe::wasm_bindgen::{prelude::*, JsCast};
use survsim_controller::{backend::SurvsimBackend, executive::Executive};
use survsim_structs::report::Report;
use survsim_webdemo::WorkerResultMessage;
use web_sys::js_sys::{Array, JsString};
use web_sys::{DedicatedWorkerGlobalScope, MessageEvent};

fn main() {
    let planner = Box::new(survsim_planner::greedy::solve_greedy_cycles);
    let mut executive = Executive::new(planner);
    let mut backend = SurvsimBackend::new();
    web_sys::console::log_1(&"worker starting".into());

    let scope = DedicatedWorkerGlobalScope::from(JsValue::from(web_sys::js_sys::global()));
    let scope_clone = scope.clone();
    let scope_clone2 = scope.clone();

    let last_report: Rc<RefCell<Option<Report>>> = Rc::new(RefCell::new(None));
    let last_report_executive_clone = last_report.clone();

    let executive_call = Closure::wrap(Box::new(move || {
        web_sys::console::log_1(&"solving".into());

        if let Some(report) = last_report_executive_clone.borrow_mut().take() {
            web_sys::console::log_1(&"actually solving".into());
            backend.set_report(report);
            let plan = executive.update(&mut backend);
            scope_clone
                .post_message(
                    &serde_json::to_string(&WorkerResultMessage::Plan(plan))
                        .unwrap()
                        .into(),
                )
                .expect("posting result message succeeds");

            backend.dispatch(|g| {
                scope_clone
                    .post_message(
                        &serde_json::to_string(&WorkerResultMessage::Goal(g))
                            .unwrap()
                            .into(),
                    )
                    .expect("posting result message succeeds");
            });
        }

        web_sys::console::log_1(&"done".into());

    }) as Box<dyn FnMut()>);

    let onmessage = Closure::wrap(Box::new(move |msg: MessageEvent| {
        web_sys::console::log_1(&"got report".into());
        let string = msg.data().as_string().unwrap();
        let report = serde_json::from_str(&string).unwrap();
        {
            *last_report.borrow_mut() = Some(report);
        }
        web_sys::console::log_1(&"saved report. setting timeout".into());

        scope_clone2
            .set_timeout_with_callback_and_timeout_and_arguments_0(
                executive_call.as_ref().unchecked_ref(),
                0,
            )
            .unwrap();
    }) as Box<dyn Fn(MessageEvent)>);

    scope.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
    onmessage.forget();

    // // The worker must send a message to indicate that it's ready to receive messages.
    // scope
    //     .post_message(&Array::new().into())
    //     .expect("posting ready message succeeds");
}
