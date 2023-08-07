use ricsek::{config::run::RunConfig, dynamics::run};

use structopt::StructOpt;

#[derive(Debug, StructOpt)]
#[structopt(name = "ricsek_run", about = "Run an initialized simulation...")]
pub struct RunCli {
    #[structopt(short = "d", long = "dt-view")]
    pub dt_view: f64,

    #[structopt(short = "t")]
    pub t_max: f64,

    #[structopt(short = "r", long = "run")]
    pub run_id: Option<usize>,
}

fn main() {
    let args = RunCli::from_args();

    let conn = &mut ricsek::db::establish_connection();

    let run_id = match args.run_id {
        Some(run_id) => run_id,
        None => {
            println!("No run_id specified, using latest run_id");
            ricsek::db::read_latest_run_id(conn)
        }
    };
    println!("Running run_id: {}", run_id);

    let setup = ricsek::db::read_run(conn, run_id);

    let run_config = RunConfig {
        t_max: args.t_max,
        dstep_view: setup.parameters.sim_params.to_steps(args.dt_view),
        run_id,
    };

    let sim_state = ricsek::db::read_latest_checkpoint(conn, run_id);
    println!("Running from step {}, t={}s", sim_state.step, sim_state.t);

    run(
        conn,
        setup.parameters.sim_params,
        sim_state,
        run_config,
    );
    println!("Done!");
}
