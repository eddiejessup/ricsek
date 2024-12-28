CREATE TABLE env (
    id SERIAL PRIMARY KEY,
    run_id INTEGER NOT NULL REFERENCES run(id),
    step INTEGER NOT NULL,
    t FLOAT NOT NULL,
    rng JSONB NOT NULL,
    step_summary JSONB,
    UNIQUE (run_id, step)
);

CREATE INDEX idx_env_run_id ON env(run_id);

CREATE TABLE agent (
    agent_id INTEGER NOT NULL,
    env_id INTEGER NOT NULL REFERENCES env(id),
    r1x FLOAT NOT NULL,
    r1y FLOAT NOT NULL,
    r1z FLOAT NOT NULL,
    r2x FLOAT NOT NULL,
    r2y FLOAT NOT NULL,
    r2z FLOAT NOT NULL,
    th1 FLOAT NOT NULL,
    th2 FLOAT NOT NULL,
    PRIMARY KEY (agent_id, env_id)
);

CREATE INDEX idx_agent_env_id ON agent(env_id);
