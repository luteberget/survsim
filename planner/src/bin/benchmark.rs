use std::fs::read_dir;

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
                let contacts = if *fields.last().unwrap() == "ctc" {
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
    let solvers = [
        ("greedy", survsim_planner::greedy::solve_greedy_cycles),
        ("greedy2", survsim_planner::greedy::solve_greedy_cycles),
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
    for (filename, instance_spec) in instance_files.iter() {
        results.push(Vec::new());
        if filename.as_os_str().to_string_lossy().ends_with(".json") {
            let _p = hprof::enter("instance");
            let problem = {
                let _p = hprof::enter("read");

                let problem: survsim_structs::problem::Problem =
                    serde_json::from_str(&read_to_string(&filename).unwrap()).unwrap();

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
                let (obj, plan) = solver_fn(&problem);
                let time = t0.elapsed().as_secs_f32();
                results.last_mut().unwrap().push(Result {
                    time,
                    obj,
                    bound: f32::NEG_INFINITY,
                });
            }
        }
    }
    println!();
    println!("# PROFILER");
    hprof::profiler().print_timing();
    println!();

    println!("# RESULTS");

    use std::io::Write;
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
}
