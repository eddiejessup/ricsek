CREATE TABLE run (
    id SERIAL PRIMARY KEY,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    parameters JSONB NOT NULL,
    obstacles JSONB NOT NULL,
    agent_initialization JSONB NOT NULL
);
