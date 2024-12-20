use std::{cell::RefCell, rc::Rc};

use survsim_structs::{plan::Plan, report::Report, GoalMsg};

pub struct SurvSimApp {
    simulator: survsim_sim::World,
    last_updated: web_time::Instant,
    plan: Rc<RefCell<Option<Plan>>>,
    pending_msgs: Rc<RefCell<Vec<GoalMsg>>>,
    publish_report: Box<dyn FnMut(&Report)>,
}

impl SurvSimApp {
    /// Called once before the first frame.
    pub fn new(
        _cc: &eframe::CreationContext<'_>,
        plan: Rc<RefCell<Option<Plan>>>,
        pending_msgs: Rc<RefCell<Vec<GoalMsg>>>,
        publish_report: Box<dyn FnMut(&Report)>,
    ) -> Self {
        SurvSimApp {
            simulator: survsim_sim::World::default(),
            last_updated: web_time::Instant::now(),
            plan,
            pending_msgs,
            publish_report,
        }
    }
}

impl eframe::App for SurvSimApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(std::time::Duration::from_millis(60));
        let time_factor = 10.0;
        let sim_dt = time_factor * self.last_updated.elapsed().as_secs_f32();
        self.last_updated = web_time::Instant::now();
        let report = self.simulator.simulate(sim_dt);
        (self.publish_report)(&report);
        for goal_msg in self.pending_msgs.borrow_mut().drain(..) {
            self.simulator.drones[goal_msg.drone].goal = goal_msg.goal;
        }
        let current_plan = self.plan.borrow();

        let plan = egui::CentralPanel::default().show(ctx, |ui| {
            survsim_viz::draw_map_and_plan(ui, current_plan.as_ref(), Some(&report));
        });
    }
}
