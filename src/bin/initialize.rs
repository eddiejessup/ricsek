use ricsek::{config, state::*};

fn main() {
    let config = config::setup::SetupConfig::parse("config.yaml").unwrap();
    config.print();
    let sim_params = &config.parameters;

    let mut rng = rand::thread_rng();
    let agents: Vec<Agent> = config.agent_initialization.iter().flat_map(|init_config| config::setup::agents::initialize_agents(
        &mut rng,
        init_config.clone(),
        sim_params.boundaries.l(),
    )).collect();
    println!("Initialized {} agents", agents.len());

    let sim_state = SimState::new(agents);

    let connection = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::initialize_run(connection, &config);
    ricsek::db::write_checkpoint(connection, run_id, &sim_state, None);
    println!("Initialized run ID {}", run_id);
}
