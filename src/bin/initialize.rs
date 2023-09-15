use log::{info, warn};
use ricsek::{config, state::*};

fn main() {
    env_logger::init();

    let config = config::setup::SetupConfig::parse("config.yaml").unwrap();
    config.print();
    let sim_params = &config.parameters;

    let mut rng = rand::thread_rng();
    warn!("{}", 1e6 * sim_params.agent_inter_sphere_length);
    let agents: Vec<Agent> = config
        .agent_initialization
        .iter()
        .flat_map(|init_config| {
            config::setup::agents::initialize_agents(
                &mut rng,
                init_config.clone(),
                sim_params.boundaries.l(),
                sim_params.agent_inter_sphere_length,
            )
        })
        .collect();
    info!("Initialized {} agents", agents.len());

    let sim_state = SimState::new(agents);

    let connection = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::initialize_run(connection, &config);
    ricsek::db::write_checkpoint(connection, run_id, &sim_state, None).unwrap();
    info!("Initialized run ID {}", run_id);
}
