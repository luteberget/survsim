use survsim_structs::{
    problem::{Dist, Poi, Problem, Vehicle},
    report::Report,
    TaskRef,
};

pub fn create_planning_problem(report: &Report) -> Problem {
    let representative_drone = &report.drones[0];
    let mut pois = Vec::new();

    for (i, _) in report.fixed_tasks.iter().enumerate() {
        pois.push(Poi {
            task_ref: TaskRef::FixedTask(i),
            reward_rate: 1.0,
            battery_rate: representative_drone.battery_consumption_hovering,
        })
    }

    for (contact_idx, contact) in report.contacts.iter().enumerate() {
        if contact.in_sight {
            pois.push(Poi {
                task_ref: TaskRef::Contact(contact_idx),
                battery_rate: representative_drone.battery_consumption_hovering,
                reward_rate: 10.0,
            });
        }
    }

    let mut vehicles = Vec::new();
    for drone in report.drones.iter() {
        vehicles.push(Vehicle {
            start_battery: drone.battery_level,
        });
    }

    let mut distances = Vec::new();
    for (a, b, dt) in report.distances.entries.iter() {
        distances.push((
            (*a, *b),
            Dist {
                dt: *dt,
                d_batt: *dt * representative_drone.battery_consumption_traveling,
            },
        ));
    }

    Problem {
        t_takeoff: report.takeoff_separation_time,
        battery_capacity: 1.0,
        pois,
        vehicles,
        distances,
    }
}
