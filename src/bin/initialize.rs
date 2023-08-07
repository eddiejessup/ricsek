use ricsek::{config, state::*};

fn main() {
    let config = config::setup::SetupConfig::parse("config.yaml").unwrap();
    config.print();
    let sim_params = &config.parameters.sim_params;

    let mut rng = rand::thread_rng();
    let agents = config::setup::agents::initialize_agents(
        &mut rng,
        &config.agent_initialization,
        sim_params.boundaries.l(),
    );
    println!("Initialized {} agents", agents.len());

    let sim_state = SimState::new(agents);

    let connection = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::initialize_run(connection, &config);
    ricsek::db::write_checkpoint(connection, run_id, &sim_state);
    println!("Initialized run ID {}", run_id);
}
