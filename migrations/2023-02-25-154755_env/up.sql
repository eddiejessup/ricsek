CREATE TABLE env (
    id SERIAL PRIMARY KEY,
    run_id INTEGER NOT NULL REFERENCES run(id),
    step INTEGER NOT NULL,
    t FLOAT NOT NULL,
    UNIQUE (run_id, step)
);

CREATE INDEX idx_env_run_id ON env(run_id);

CREATE TABLE agent (
    agent_id INTEGER NOT NULL,
    env_id INTEGER NOT NULL REFERENCES env(id),
    rx FLOAT NOT NULL,
    ry FLOAT NOT NULL,
    rz FLOAT NOT NULL,
    ux FLOAT NOT NULL,
    uy FLOAT NOT NULL,
    uz FLOAT NOT NULL,
    step_summary JSONB,
    PRIMARY KEY (agent_id, env_id)
);

CREATE INDEX idx_agent_env_id ON agent(env_id);
