use paho_mqtt::Message;
use survsim_sim::World;
use survsim_structs::{Goal, GoalMsg};


fn main() {
    let mqtt_opts = paho_mqtt::CreateOptionsBuilder::new()
        .server_uri("mqtt://localhost:1883")
        .finalize();
    let mqtt_cli = paho_mqtt::Client::new(mqtt_opts).unwrap();
    let conn_opts = paho_mqtt::ConnectOptionsBuilder::new()
        .keep_alive_interval(std::time::Duration::from_secs(20))
        .finalize();
    mqtt_cli.connect(conn_opts).unwrap();
    mqtt_cli.subscribe("/survsim/goal", 1).unwrap();
    mqtt_cli.subscribe("/survsim/time_factor", 1).unwrap();
    let mqtt_rx = mqtt_cli.start_consuming();

    let mut world = World::default();
    // world.drones[0].goal = Goal::TaskRef(TaskRef::FixedTask(0));
    let mut last_updated = std::time::Instant::now();
    let update_frequency = std::time::Duration::from_millis(10);

    println!("survsim_sim main loop starting.");
    let mut time_factor = 1.0;
    loop {
        let sim_dt = time_factor * last_updated.elapsed().as_secs_f32();
        // println!("updateing {}", sim_dt);
        let report = world.simulate(sim_dt);
        // println!("report {:?}", report);
        mqtt_cli
            .publish(Message::new(
                "/survsim/state",
                serde_json::to_string(&report).unwrap(),
                1,
            ))
            .unwrap();

        last_updated = std::time::Instant::now();
        loop {
            let timeout = update_frequency - last_updated.elapsed();
            // println!("recv with timeout {:?}", timeout);
            match mqtt_rx.recv_timeout(timeout) {
                Ok(None) => {
                    println!("none");
                }
                Ok(Some(msg)) => {
                    if msg.topic() == "/survsim/goal" {
                        match serde_json::from_slice::<GoalMsg>(msg.payload_str().as_bytes()) {
                            Ok(goal_msg) => {
                                if goal_msg.goal == Goal::Wait && !world.drones[goal_msg.drone].base_ground.eq_xyz(&world.drones[goal_msg.drone].curr_loc) {
                                    // panic!("WAIT goal?");
                                }
                                world.drones[goal_msg.drone].goal = goal_msg.goal;
                            }
                            Err(x) => {
                                println!("WARNING: received malformed message {:?}", x);
                            }
                        }
                    } else if msg.topic() == "/survsim/time_factor" {
                        match serde_json::from_slice::<f32>(msg.payload_str().as_bytes()) {
                            Ok(x) => {
                                time_factor = x;
                            }
                            Err(x) => {
                                println!("WARNING: received malformed message {:?}", x);
                            }
                        }
                    } else {
                        panic!("unknown topic {}", msg.topic());
                    }
                }
                Err(e) => {
                    if e.is_disconnected() {
                        panic!("mqtt disconnected");
                    } else if e.is_timeout() {
                        break;
                    }
                }
            }
        }
    }
}
