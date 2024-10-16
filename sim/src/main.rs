use paho_mqtt::Message;
use survsim_structs::{
    report::{ContactReport, DistanceList, DroneReport, FixedTaskReport, Location, Report},
    Goal, GoalMsg, Point, TaskRef,
};

pub struct FixedTaskState {
    pub loc: Point,
}

pub struct ContactState {
    pub waypoints: Vec<Point>,
    pub curr_waypoint: usize,
    pub curr_loc: Point,
    pub velocity: f32,
}

pub struct DroneState {
    pub base_air: Point,
    pub base_ground: Point,
    pub curr_loc: Point,
    pub velocity: f32,
    pub battery_level: f32,
    pub battery_consumption_traveling: f32,
    pub battery_consumption_hovering: f32,
    pub goal: Goal,
}

pub struct World {
    curr_time: f32,
    drones: Vec<DroneState>,
    contacts: Vec<ContactState>,
    fixed_tasks: Vec<FixedTaskState>,
}

impl World {
    pub fn set_drone_goal(&mut self, drone_idx: usize, goal: Goal) {
        self.drones[drone_idx].goal = goal;
    }

    pub fn tiny() -> World {
     // Default hard-coded scenario setup

     let base_air = Point {
        x: 180.0,
        y: 600.0 - 540.0,
        z: 0.0,
    };
    let base_ground = Point {
        z: -50.0,
        ..base_air
    };

    let fixed_tasks = vec![
        FixedTaskState {
            loc: Point {
                x: 66.0,
                y: 600.0 - 300.0,
                z: 0.0,
            },
        },
    ];

    // Typical distances are 400. Should travel there in 5 minutes = 300 sec. Velocity: 1.3

    let drones = (0..1)
        .map(|_| DroneState {
            base_air,
            base_ground,
            battery_consumption_hovering: 0.00037, // 45 minutes on full battery
            battery_consumption_traveling: 0.0011, // 15 minutes on full battery
            battery_level: 1.0,
            curr_loc: base_ground,
            goal: Goal::Wait,
            velocity: 1.3,
        })
        .collect();

    World {
        curr_time: 0.0,
        fixed_tasks,
        drones,
        contacts: Default::default(),
    }   
    }

    pub fn new() -> World {
        // Default hard-coded scenario setup

        let base_air = Point {
            x: 180.0,
            y: 600.0 - 540.0,
            z: 0.0,
        };
        let base_ground = Point {
            z: -50.0,
            ..base_air
        };

        let fixed_tasks = vec![
            FixedTaskState {
                loc: Point {
                    x: 66.0,
                    y: 600.0 - 300.0,
                    z: 0.0,
                },
            },
            FixedTaskState {
                loc: Point {
                    x: 312.0,
                    y: 600.0 - 260.0,
                    z: 0.0,
                },
            },
            FixedTaskState {
                loc: Point {
                    x: 543.0,
                    y: 600.0 - 350.0,
                    z: 0.0,
                },
            },
            FixedTaskState {
                loc: Point {
                    x: 666.0,
                    y: 600.0 - 533.0,
                    z: 0.0,
                },
            },
        ];

        // let fixed_dists = FixedDists {
        //     fixed_task_base_dist: fixed_tasks
        //         .iter()
        //         .map(|p| p.loc.dist(&base_ground))
        //         .collect(),
        //     fixed_task_dists: (0..fixed_tasks.len())
        //         .map(|i| {
        //             (0..fixed_tasks.len())
        //                 .map(|j| fixed_tasks[i].loc.dist(&fixed_tasks[j].loc))
        //                 .collect()
        //         })
        //         .collect(),
        // };

        let p1 = vec![
            Point {
                x: 53.0,
                y: 600.0 - 25.0,
                z: 0.0,
            },
            Point {
                x: 118.0,
                y: 600.0 - 350.0,
                z: 0.0,
            },
        ];
        let p2 = vec![
            Point {
                x: 350.0,
                y: 600.0 - 25.0,
                z: 0.0,
            },
            Point {
                x: 470.0,
                y: 600.0 - 50.0,
                z: 0.0,
            },
            Point {
                x: 370.0,
                y: 600.0 - 320.0,
                z: 0.0,
            },
        ];
        let p3 = vec![
            Point {
                x: 566.0,
                y: 600.0 - 75.0,
                z: 0.0,
            },
            Point {
                x: 560.0,
                y: 600.0 - 400.0,
                z: 0.0,
            },
        ];
        let contacts = vec![
            ContactState {
                waypoints: p1.clone(),
                curr_waypoint: 0,
                curr_loc: p1[0],
                velocity: 0.5,
            },
            ContactState {
                waypoints: p2.clone(),
                curr_waypoint: 0,
                curr_loc: p2[0],
                velocity: 0.5,
            },
            ContactState {
                waypoints: p3.clone(),
                curr_waypoint: 0,
                curr_loc: p3[0],
                velocity: 0.5,
            },
        ];

        let drones = (0..6)
            .map(|_| DroneState {
                base_air,
                base_ground,
                battery_consumption_hovering: 0.00037, // 45 minutes on full battery
                battery_consumption_traveling: 0.0011, // 15 minutes on full battery
                battery_level: 1.0,
                curr_loc: base_ground,
                goal: Goal::Wait,
                velocity: 1.3,
                })
            .collect();

        World {
            curr_time: 0.0,
            fixed_tasks,
            drones,
            contacts,
        }
    }

    pub fn simulate(&mut self, dt: f32) -> Report {
        const SIGHTING_DIST: f32 = 90.0;
        let base = self.drones[0].base_air;

        fn go_towards(max_dist: &mut f32, source: &mut Point, target: Point) -> bool {
            let dx = target.x - source.x;
            let dy = target.y - source.y;
            let dz = target.z - source.z;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist <= *max_dist {
                *max_dist -= dist;
                *source = target;
                true
            } else {
                // dist is greater than dt
                let scaling = *max_dist / dist;
                source.x += dx * scaling;
                source.y += dy * scaling;
                source.z += dz * scaling;
                *max_dist = 0.0;
                false
            }
        }

        // First, move the contacts
        for c in self.contacts.iter_mut() {
            let mut dist = dt * c.velocity;
            while go_towards(&mut dist, &mut c.curr_loc, c.waypoints[c.curr_waypoint]) {
                c.curr_waypoint = (c.curr_waypoint + 1) % c.waypoints.len();
            }
        }

        // Then move the drones
        for d in self.drones.iter_mut() {
            let mut remaining_dist = dt * d.velocity;
            let on_ground = d.curr_loc.eq_xyz(&d.base_ground);
            let at_base = d.curr_loc.eq_xy(&d.base_air);

            let flying = match &d.goal {
                Goal::Wait => false,
                Goal::Base => {
                    let descend =
                        at_base || go_towards(&mut remaining_dist, &mut d.curr_loc, d.base_air);

                    if descend && !on_ground {
                        go_towards(&mut remaining_dist, &mut d.curr_loc, d.base_ground);
                    }
                    !on_ground
                }
                Goal::TaskRef(task_ref) => {
                    let ascending = at_base
                        && d.curr_loc.dist(&d.base_ground) / d.curr_loc.dist(&d.base_air) < 10.0;

                    let fly =
                        !ascending || go_towards(&mut remaining_dist, &mut d.curr_loc, d.base_air);

                    let arrived = if fly {
                        let task_loc = match task_ref {
                            TaskRef::FixedTask(t) => self.fixed_tasks[*t].loc,
                            TaskRef::Contact(c) => self.contacts[*c].curr_loc,
                        };
                        go_towards(&mut remaining_dist, &mut d.curr_loc, task_loc)
                    } else {
                        false
                    };

                    !arrived
                }
            };

            if !on_ground {
                d.battery_level -= dt
                    * if flying {
                        d.battery_consumption_traveling
                    } else {
                        d.battery_consumption_hovering
                    };
            } else {
                d.battery_level = 1.0;
            }
        }

        // Update time
        self.curr_time += dt;

        // Create report
        let mut contacts_in_sight: Vec<usize> = Vec::new();

        let drones = self
            .drones
            .iter()
            .map(|d| {
                let at_base = d.curr_loc.eq_xy(&d.base_air);
                let on_ground = d.curr_loc.eq_xyz(&d.base_ground);

                let fixed_task_dists = self
                    .fixed_tasks
                    .iter()
                    .map(|ft| d.curr_loc.dist(&ft.loc))
                    .collect::<Vec<_>>();

                let cs = self.contacts.iter().enumerate().filter_map(|(i, c)| {
                    (d.curr_loc.dist(&c.curr_loc) <= SIGHTING_DIST).then_some(TaskRef::Contact(i))
                });

                let tasks_in_sight = fixed_task_dists
                    .iter()
                    .enumerate()
                    .filter_map(|(i, d)| (*d <= SIGHTING_DIST).then_some(TaskRef::FixedTask(i)))
                    .chain(cs)
                    .collect::<Vec<_>>();

                for t in tasks_in_sight.iter() {
                    if let TaskRef::Contact(c) = t {
                        if !contacts_in_sight.contains(c) {
                            contacts_in_sight.push(*c);
                        }
                    }
                }

                DroneReport {
                    at_base,
                    base: d.base_air,
                    is_airborne: !on_ground,
                    goal: d.goal,
                    loc: d.curr_loc,
                    battery_level: d.battery_level,
                    battery_consumption_traveling: d.battery_consumption_traveling,
                    battery_consumption_hovering: d.battery_consumption_hovering,
                    tasks_in_sight,
                }
            })
            .collect::<Vec<_>>();

        for d in self.drones.iter_mut() {
            // Disable goals for  non-sighted contacts
            if let Goal::TaskRef(TaskRef::Contact(c)) = d.goal {
                if !contacts_in_sight.contains(&c) {
                    d.goal = Goal::Wait;
                }
            }
        }

        let contacts = self
            .contacts
            .iter()
            .enumerate()
            .map(|(i, c)| ContactReport {
                in_sight: contacts_in_sight.contains(&i),
                loc: c.curr_loc,
            })
            .collect();

        let mut distances = DistanceList {
            entries: Default::default(),
        };

        let velocity = self.drones[0].velocity;

        // Base to fixed, and fixed to fixed.
        for (i, f1) in self.fixed_tasks.iter().enumerate() {
            distances.entries.push((
                Location::Base,
                Location::Task(TaskRef::FixedTask(i)),
                base.dist(&f1.loc) / velocity,
            ));
            for (j, f2) in self.fixed_tasks.iter().enumerate() {
                if i < j {
                    distances.entries.push((
                        Location::Task(TaskRef::FixedTask(i)),
                        Location::Task(TaskRef::FixedTask(j)),
                        f1.loc.dist(&f2.loc) / velocity,
                    ));
                }
            }
        }

        // Contact to base, contact to contact, and fixed to contact.
        for (i, c1) in self.contacts.iter().enumerate() {
            distances.entries.push((
                Location::Base,
                Location::Task(TaskRef::Contact(i)),
                base.dist(&c1.curr_loc) / velocity,
            ));
            for (j, c2) in self.contacts.iter().enumerate() {
                if i < j {
                    distances.entries.push((
                        Location::Task(TaskRef::Contact(i)),
                        Location::Task(TaskRef::Contact(j)),
                        c1.curr_loc.dist(&c2.curr_loc) / velocity,
                    ));
                }
            }
            for (j, f) in self.fixed_tasks.iter().enumerate() {
                distances.entries.push((
                    Location::Task(TaskRef::Contact(i)),
                    Location::Task(TaskRef::FixedTask(j)),
                    f.loc.dist(&c1.curr_loc) / velocity,
                ));
            }
        }

        // Drone to base, drone to fixed and drone to contact
        for (i, d) in self.drones.iter().enumerate() {
            distances.entries.push((
                Location::Base,
                Location::DroneInitial(i),
                base.dist(&d.curr_loc) / velocity,
            ));

            for (j, f) in self.fixed_tasks.iter().enumerate() {
                distances.entries.push((
                    Location::Task(TaskRef::FixedTask(j)),
                    Location::DroneInitial(i),
                    f.loc.dist(&d.curr_loc) / velocity,
                ));
            }

            for (j, c) in self.contacts.iter().enumerate() {
                distances.entries.push((
                    Location::Task(TaskRef::Contact(j)),
                    Location::DroneInitial(i),
                    c.curr_loc.dist(&d.curr_loc) / velocity,
                ));
            }
        }

        let fixed_tasks = self
            .fixed_tasks
            .iter()
            .map(|t| FixedTaskReport { loc: t.loc })
            .collect();

        Report {
            current_time: self.curr_time,
            takeoff_separation_time: 5.0,
            drones,
            contacts,
            distances,
            fixed_tasks,
        }
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}

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
    let mqtt_rx = mqtt_cli.start_consuming();

    let mut world = World::default();
    // world.drones[0].goal = Goal::TaskRef(TaskRef::FixedTask(0));
    let mut last_updated = std::time::Instant::now();
    let update_frequency = std::time::Duration::from_millis(10);

    println!("survsim_sim main loop starting.");
    loop {
        const SIM_SPEED: f32 = 10.0;

        let sim_dt = SIM_SPEED * last_updated.elapsed().as_secs_f32();
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
                    match serde_json::from_slice::<GoalMsg>(msg.payload_str().as_bytes()) {
                        Ok(goal_msg) => {
                            world.drones[goal_msg.drone].goal = goal_msg.goal;
                        }
                        Err(x) => {
                            println!("WARNING: received malformed message {:?}", x);
                        }
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
