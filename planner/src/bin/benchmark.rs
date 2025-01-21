use std::{error::Error, fs::read_dir, path::PathBuf};

#[cfg(not(feature = "prof"))]
pub fn main() {
    println!("benchmarks not supported -- enable 'prof' crate feature")
}

struct InstanceSpec {
    name: String,
    contacts: usize,
    vehicles: usize,
    idx: usize,
}

#[derive(Debug)]
struct Result {
    time: f32,
    obj: f32,
    bound: f32,
}

fn get_instance_files() -> std::vec::Vec<(std::path::PathBuf, InstanceSpec)> {
    let mut filenames = read_dir("bench")
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|e| e.as_os_str().to_string_lossy().ends_with(".json"))
        .collect::<Vec<_>>();
    filenames.sort();

    let instance_files = filenames
        .into_iter()
        .map(|filename| {
            let (name, contacts, vehicles, idx) = {
                let split = filename.file_stem().unwrap().to_string_lossy();
                let mut fields = split.split("_").collect::<Vec<_>>();
                let idx = fields.pop().unwrap().parse::<usize>().unwrap();
                let vehicles = fields.pop().unwrap()[1..].parse::<usize>().unwrap();
                let contacts = if fields.last().unwrap().starts_with("ctc") {
                    fields.pop().unwrap()[3..].parse::<usize>().unwrap_or(1)
                } else {
                    0
                };
                let name = fields.pop().unwrap();
                (name.to_string(), contacts, vehicles, idx)
            };
            (
                filename,
                InstanceSpec {
                    name,
                    contacts,
                    vehicles,
                    idx,
                },
            )
        })
        .collect::<Vec<_>>();
    instance_files
}

#[cfg(feature = "prof")]
pub fn main() {
    use core::f32;
    use std::{fs::read_to_string, time::Instant};
    type Solver = fn(&Problem) -> ((f32, f32), Plan);

    fn gurobi_5sec(problem: &Problem) -> ((f32, f32), Plan) {
        survsim_planner::milp::solve::<GurobiSolver>(problem, 5.0, None, false)
    }
    fn gurobi_30sec(problem: &Problem) -> ((f32, f32), Plan) {
        survsim_planner::milp::solve::<GurobiSolver>(problem, 30.0, None, false)
    }
    fn highs_5sec(problem: &Problem) -> ((f32, f32), Plan) {
        survsim_planner::milp::solve::<HighsSolverInstance>(problem, 5.0, None, false)
    }
    fn highs_30sec(problem: &Problem) -> ((f32, f32), Plan) {
        survsim_planner::milp::solve::<HighsSolverInstance>(problem, 30.0, None, false)
    }

    fn gurobi_5sec_greedymipstart(problem: &Problem) -> ((f32, f32), Plan) {
        let (_, _, init) = survsim_planner::greedy::solve_greedy_cycles(problem, false);
        survsim_planner::milp::solve::<GurobiSolver>(problem, 5.0, Some(&init), false)
    }
    fn gurobi_30sec_greedymipstart(problem: &Problem) -> ((f32, f32), Plan) {
        let (_, _, init) = survsim_planner::greedy::solve_greedy_cycles(problem, false);
        survsim_planner::milp::solve::<GurobiSolver>(problem, 30.0, Some(&init), false)
    }
    fn highs_5sec_greedymipstart(problem: &Problem) -> ((f32, f32), Plan) {
        let (_, _, init) = survsim_planner::greedy::solve_greedy_cycles(problem, false);
        survsim_planner::milp::solve::<HighsSolverInstance>(problem, 5.0, Some(&init), false)
    }
    fn highs_30sec_greedymipstart(problem: &Problem) -> ((f32, f32), Plan) {
        let (_, _, init) = survsim_planner::greedy::solve_greedy_cycles(problem, false);
        survsim_planner::milp::solve::<HighsSolverInstance>(problem, 30.0, Some(&init), false)
    }

    fn colgen_5sec(problem: &Problem) -> ((f32, f32), Plan) {
        survsim_planner::colgen::HeuristicColgenSolver::new(problem).solve_price_and_branch(5.0)
    }
    fn colgen_30sec(problem: &Problem) -> ((f32, f32), Plan) {
        survsim_planner::colgen::HeuristicColgenSolver::new(problem).solve_price_and_branch(30.0)
    }
    fn colgen_5sec_greedyinit(problem: &Problem) -> ((f32, f32), Plan) {
        survsim_planner::colgen::HeuristicColgenSolver::new(problem)
            .add_greedy_columns()
            .solve_price_and_branch(5.0)
    }
    fn colgen_30sec_greedyinit(problem: &Problem) -> ((f32, f32), Plan) {
        survsim_planner::colgen::HeuristicColgenSolver::new(problem)
            .add_greedy_columns()
            .solve_price_and_branch(30.0)
    }

    fn greedy_fast(problem: &Problem) -> ((f32, f32), Plan) {
        let (b, p, _) = survsim_planner::greedy::solve_greedy_cycles(problem, false);
        (b, p)
    }

    fn greedy_verify(problem: &Problem) -> ((f32, f32), Plan) {
        let (b, p, _) = survsim_planner::greedy::solve_greedy_cycles(problem, true);
        (b, p)
    }

    let solvers: Vec<(&str, Solver)> = vec![
        ("greedy", greedy_fast),
        ("greedy_verify", greedy_verify),
        ("colgen_5s", colgen_5sec),
        ("colgen_30s", colgen_30sec),
        ("colgen_5s_gi", colgen_5sec_greedyinit),
        ("colgen_30s_gi", colgen_30sec_greedyinit),
        ("gurobi_5s", gurobi_5sec),
        ("gurobi_30s", gurobi_30sec),
        ("highs_5s", highs_5sec),
        ("highs_30s", highs_30sec),
        ("gurobi_5s_gi", gurobi_5sec_greedymipstart),
        ("gurobi_30s_gi", gurobi_30sec_greedymipstart),
        ("highs_5s_gi", highs_5sec_greedymipstart),
        ("highs_30s_gi", highs_30sec_greedymipstart),
    ];

    println!("---------------------------");
    println!("survsim solver benchmarking");
    println!("---------------------------");
    println!();
    println!(
        "  solvers: {}",
        solvers
            .iter()
            .map(|(name, _)| *name)
            .collect::<Vec<_>>()
            .join(", ")
    );
    println!();

    let instance_files = get_instance_files();
    let mut results: Vec<Vec<Result>> = Vec::new();

    println!("# RUNNING {} INSTANCES", instance_files.len());
    for (filename, _instance_spec) in instance_files.iter() {
        results.push(Vec::new());
        if filename.as_os_str().to_string_lossy().ends_with(".json") {
            let _p = hprof::enter("instance");
            let problem = {
                let _p = hprof::enter("read");

                let problem: survsim_structs::problem::Problem =
                    serde_json::from_str(&read_to_string(filename).unwrap()).unwrap();

                println!(
                    " * instance {} with {} vehicles {} pois",
                    filename.display(),
                    problem.vehicles.len(),
                    problem.pois.len()
                );
                problem
            };

            for (solver_name, solver_fn) in &solvers {
                println!("   - solving with: \"{}\"", solver_name);
                let _p0 = hprof::enter("plan");
                let t0 = Instant::now();
                let ((obj, bound), _plan) = solver_fn(&problem);
                let time = t0.elapsed().as_secs_f32();
                results
                    .last_mut()
                    .unwrap()
                    .push(Result { time, obj, bound });
            }
        }
    }
    println!();
    println!("# PROFILER");
    hprof::profiler().print_timing();
    println!();

    println!("# RESULTS");

    use std::io::Write;

    use survsim_planner::extsolvers::gurobi::GurobiSolver;
    use survsim_planner::extsolvers::highs::HighsSolverInstance;
    use survsim_structs::plan::Plan;
    use survsim_structs::problem::Problem;
    let table = Vec::new();
    let mut tablewriter = tabwriter::TabWriter::new(table);
    write!(&mut tablewriter, "filename\tctc\tvhs\tidx").unwrap();
    for (solver_name, _) in &solvers {
        write!(&mut tablewriter, "\t|\t{}\t\t", solver_name).unwrap();
    }
    writeln!(&mut tablewriter).unwrap();

    write!(&mut tablewriter, "\t\t\t").unwrap();
    for (_solver_name, _) in &solvers {
        write!(&mut tablewriter, "\t|\ttime\tobj\tbnd",).unwrap();
    }
    writeln!(&mut tablewriter).unwrap();

    write!(&mut tablewriter, "---\t---\t---\t---").unwrap();
    for _ in &solvers {
        write!(&mut tablewriter, "\t\t---\t---\t---").unwrap();
    }
    writeln!(&mut tablewriter).unwrap();

    for ((_filename, instance), solver_results) in instance_files.iter().zip(results.iter()) {
        write!(
            &mut tablewriter,
            "{}\t{}\t{}\t{}",
            instance.name, instance.contacts, instance.vehicles, instance.idx
        )
        .unwrap();
        for ((_solver_name, _), result) in solvers.iter().zip(solver_results.iter()) {
            write!(
                &mut tablewriter,
                "\t|\t{:.2}\t{:.0}\t{:.0}",
                result.time, result.obj, result.bound
            )
            .unwrap();
        }
        writeln!(&mut tablewriter).unwrap();
    }

    let written = String::from_utf8(tablewriter.into_inner().unwrap()).unwrap();
    println!("{}", written);

    write_csv(
        &solvers.iter().map(|(n, _)| *n).collect::<Vec<_>>(),
        &instance_files,
        &results,
    )
    .unwrap();
}

fn write_csv(
    solver_names: &[&str],
    instance_files: &[(PathBuf, InstanceSpec)],
    results: &[Vec<Result>],
) -> std::result::Result<(), Box<dyn Error>> {
    let mut wtr = csv::Writer::from_path("benchmark.csv")?;
    let mut header: Vec<String> = vec![
        "filename".to_string(),
        "contacts".to_string(),
        "vehicles".to_string(),
        "instance_idx".to_string(),
    ];
    for solver in solver_names.iter() {
        header.push(format!("{}--time", solver));
        header.push(format!("{}--obj", solver));
        header.push(format!("{}--bound", solver));
    }

    wtr.write_record(&header)?;

    for ((_filename, instance), solver_results) in instance_files.iter().zip(results.iter()) {
        let mut row: Vec<String> = vec![
            instance.name.to_string(),
            format!("{}", instance.contacts),
            format!("{}", instance.vehicles),
            format!("{}", instance.idx),
        ];
        for (_solver_name, result) in solver_names.iter().zip(solver_results.iter()) {
            row.push(format!("{:.2}", result.time));
            row.push(format!("{:.2}", result.obj));
            row.push(format!("{:.2}", result.bound));
        }

        wtr.write_record(&row)?;
    }

    wtr.flush()?;
    Ok(())
}
