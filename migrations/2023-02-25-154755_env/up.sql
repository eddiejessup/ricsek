CREATE TABLE env (
    id SERIAL PRIMARY KEY,
    run_id INTEGER NOT NULL REFERENCES run(id),
    step INTEGER NOT NULL,
    t FLOAT NOT NULL
);

CREATE TABLE agent (
    agent_id INTEGER NOT NULL,
    env_id INTEGER NOT NULL REFERENCES env(id),
    rx FLOAT NOT NULL,
    ry FLOAT NOT NULL,
    ux FLOAT NOT NULL,
    uy FLOAT NOT NULL,
    PRIMARY KEY (agent_id, env_id)
);
