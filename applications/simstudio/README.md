
# Sim Studio

[Sim](https://github.com/simstudioai/sim) AI-workflow builder, pinned to a
release (`SIM_REF` build argument) and run in development mode.

```bash
make -C applications build_simstudio
docker compose -f compositions/simstudio.yaml up
```

The app is served on `http://localhost:3000` (realtime server on port 3002).
Database migrations run when the container starts, against the `db` service
(Postgres with pgvector). Local LLMs come from the optional Ollama services;
models are pulled at runtime, not baked into the image:

```bash
docker compose -f compositions/simstudio.yaml --profile local-gpu up -d   # or local-cpu
docker compose -f compositions/simstudio.yaml exec local-llm-gpu ollama pull qwen3:8b
```
