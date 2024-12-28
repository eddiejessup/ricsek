use clap::Parser;
use log::{info, warn};
use ricsek::{
    config::{self, run::RunParams},
    dynamics::run,
    state::{Agent, SimState},
};

#[derive(Debug, clap::Parser)]
#[command(name = "ricsek_run", about = "Run an initialized simulation...")]
pub struct RunCli {
    #[arg(short = 'd', long = "dt-view")]
    pub dt_view: f64,

    #[arg(short = 't')]
    pub t_max: f64,

    #[arg(short = 'r', long = "run")]
    pub run_id: Option<usize>,

    #[arg(long = "resume")]
    pub resume: bool,
}

fn main() {
    env_logger::init();
    let args = RunCli::parse();

    let conn = &mut ricsek::db::establish_connection();

    let (run_id, setup_config, sim_state) = if args.resume {
        info!("Resuming run");
        let run_id = match args.run_id {
            Some(run_id) => run_id,
            None => {
                warn!("No run_id specified, using latest run_id");
                ricsek::db::read_latest_run_id(conn)
            }
        };
        info!("Resuming run_id: {}", run_id);

        let setup_config = ricsek::db::read_run(conn, run_id);

        let sim_state = ricsek::db::read_latest_checkpoint(conn, run_id);
        info!("Resuming from step {}, t={}s", sim_state.step, sim_state.t);

        (run_id, setup_config, sim_state)
    } else {
        info!("Initializing new run");

        let setup_config = config::setup::SetupConfig::parse("config.yaml").unwrap();
        setup_config.print();

        let mut rng = rand::thread_rng();
        let agents: Vec<Agent> = setup_config
            .agent_initialization
            .iter()
            .flat_map(|init_config| {
                config::setup::agents::initialize_agents(
                    &mut rng,
                    init_config.clone(),
                    setup_config.parameters.boundaries.l(),
                    setup_config.parameters.agent_inter_sphere_length,
                )
            })
            .collect();
        info!("Initialized {} agents", agents.len());

        let sim_state = SimState::new(agents);

        let run_id = ricsek::db::initialize_run(conn, &setup_config);
        info!("Initialized run ID {}", run_id);

        ricsek::db::write_checkpoint(conn, run_id, &sim_state, None).unwrap();

        (run_id, setup_config, sim_state)
    };

    let run_config = RunParams {
        t_max: args.t_max,
        dstep_view: setup_config.parameters.to_steps(args.dt_view),
        run_id,
    };

    run(
        conn,
        setup_config.parameters,
        setup_config.sample_points,
        sim_state,
        run_config,
    )
    .unwrap();
    info!("Done!");
}
