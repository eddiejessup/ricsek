use log::{info, warn};
use ricsek::{
    config::{self, run::RunConfig},
    dynamics::run,
    state::{Agent, SimState},
};

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

    #[structopt(long="resume")]
    pub resume: bool,
  }

fn main() {
    env_logger::init();
    let args = RunCli::from_args();

    let conn = &mut ricsek::db::establish_connection();

    let (run_id, parameters, sim_state) = if args.resume {
        info!("Resuming run");
        let run_id = match args.run_id {
            Some(run_id) => run_id,
            None => {
                warn!("No run_id specified, using latest run_id");
                ricsek::db::read_latest_run_id(conn)
            }
        };
        info!("Resuming run_id: {}", run_id);

        let setup = ricsek::db::read_run(conn, run_id);

        let sim_state = ricsek::db::read_latest_checkpoint(conn, run_id);
        info!("Resuming from step {}, t={}s", sim_state.step, sim_state.t);

        (run_id, setup.parameters, sim_state)
    } else {
        info!("Initializing new run");

        let config = config::setup::SetupConfig::parse("config.yaml").unwrap();
        config.print();

        let mut rng = rand::thread_rng();
        let agents: Vec<Agent> = config
            .agent_initialization
            .iter()
            .flat_map(|init_config| {
                config::setup::agents::initialize_agents(
                    &mut rng,
                    init_config.clone(),
                    config.parameters.boundaries.l(),
                    config.parameters.agent_inter_sphere_length,
                )
            })
            .collect();
        info!("Initialized {} agents", agents.len());

        let sim_state = SimState::new(agents);

        let run_id = ricsek::db::initialize_run(conn, &config);
        info!("Initialized run ID {}", run_id);

        ricsek::db::write_checkpoint(conn, run_id, &sim_state, None).unwrap();

        (run_id, config.parameters, sim_state)
    };

    let run_config = RunConfig {
        t_max: args.t_max,
        dstep_view: parameters.to_steps(args.dt_view),
        run_id,
    };

    run(conn, parameters, sim_state, run_config).unwrap();
    info!("Done!");
}
