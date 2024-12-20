#[cfg(test)]
mod tests {
// #[test]
// pub fn test() {
//     let _ = env_logger::try_init();
//     let problem =
//         serde_json::from_str(&std::fs::read_to_string("../first_problem.json").unwrap()).unwrap();
//     let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
//     hprof::profiler().print_timing();
//     x.print();
// }

use survsim_structs::plan::Plan;

use crate::colgen::HeuristicColgenSolver;
use crate::greedy::solve_greedy_cycles;

#[test]
pub fn test_flying1() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression1.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying2() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression2.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying3() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression3.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying4() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression4.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying5() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression5.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_flying6() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression6.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_toocomplicated1() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../no_problem.json").unwrap()).unwrap();
    let _x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_toocomplicated2() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../a_problem.json").unwrap()).unwrap();
    let _x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_regression7() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression7.json").unwrap()).unwrap();
    let _x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_regression8() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression8_.json").unwrap()).unwrap();
    let _x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_regression9() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression9.json").unwrap()).unwrap();
    let _x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_regression10() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression10.json").unwrap()).unwrap();
    let x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    x.print();
}


#[test]
pub fn test_regression11() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression11.json").unwrap()).unwrap();
    let x = solve_greedy_cycles(&problem);
    hprof::profiler().print_timing();
    x.print();
}

#[test]
pub fn test_regression12() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../regression12.json").unwrap()).unwrap();
    let x = solve_greedy_cycles(&problem);
    hprof::profiler().print_timing();
    x.print();
}


#[test]
pub fn test_toocomplicated3() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../problem_1730299110063.json").unwrap()).unwrap();
    let _x = HeuristicColgenSolver::new(&problem).solve_heuristic();
    hprof::profiler().print_timing();
    // x.print();
}

pub fn send_plan(plan: &Plan) {
    let mqtt_opts = paho_mqtt::CreateOptionsBuilder::new()
        .server_uri("mqtt://localhost:1883")
        .finalize();
    let mqtt_cli = paho_mqtt::Client::new(mqtt_opts).unwrap();
    let conn_opts = paho_mqtt::ConnectOptionsBuilder::new()
        .keep_alive_interval(std::time::Duration::from_secs(20))
        .finalize();
    mqtt_cli.connect(conn_opts).unwrap();

    mqtt_cli
        .publish(paho_mqtt::Message::new(
            "/survsim/plan",
            serde_json::to_string(&plan).unwrap(),
            1,
        ))
        .unwrap();
}

#[test]
pub fn test_rp1() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../r_p1.json").unwrap()).unwrap();
    let p = HeuristicColgenSolver::new(&problem).solve_heuristic();
    send_plan(&p);
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_rp2() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../r_p2.json").unwrap()).unwrap();

    let p = HeuristicColgenSolver::new(&problem).solve_heuristic();
    send_plan(&p);
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_rd1() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../r_d1.json").unwrap()).unwrap();
    let p = HeuristicColgenSolver::new(&problem).solve_heuristic();
    send_plan(&p);
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_rd2() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../r_d2.json").unwrap()).unwrap();

    let p = HeuristicColgenSolver::new(&problem).solve_heuristic();
    send_plan(&p);
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_r41() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../r_41.json").unwrap()).unwrap();
    let p = HeuristicColgenSolver::new(&problem).solve_heuristic();
    send_plan(&p);
    hprof::profiler().print_timing();
    // x.print();
}

#[test]
pub fn test_r42() {
    let _ = env_logger::try_init();
    let problem = serde_json::from_str(&std::fs::read_to_string("../r_42.json").unwrap()).unwrap();

    let p = HeuristicColgenSolver::new(&problem).solve_heuristic();
    send_plan(&p);
    hprof::profiler().print_timing();
    // x.print();
}
}