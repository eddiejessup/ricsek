// @generated automatically by Diesel CLI.

diesel::table! {
    agent (agent_id, env_id) {
        agent_id -> Int4,
        env_id -> Int4,
        rx -> Float8,
        ry -> Float8,
        rz -> Float8,
        ux -> Float8,
        uy -> Float8,
        uz -> Float8,
    }
}

diesel::table! {
    env (id) {
        id -> Int4,
        run_id -> Int4,
        step -> Int4,
        t -> Float8,
    }
}

diesel::table! {
    run (id) {
        id -> Int4,
        created_at -> Timestamptz,
        parameters -> Jsonb,
        agent_initialization -> Jsonb,
    }
}

diesel::joinable!(agent -> env (env_id));
diesel::joinable!(env -> run (run_id));

diesel::allow_tables_to_appear_in_same_query!(
    agent,
    env,
    run,
);
