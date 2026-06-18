# Marimo

Containerized [Marimo](https://marimo.io/) notebook server.

The container starts Marimo in edit mode on port `2718` and uses `/notebooks` as its workspace.

```bash
docker compose -f compositions/marimo.yaml up
```

Open:

```text
http://localhost:2718
```

By default the compose service mounts `applications/marimo/notebooks` into `/notebooks`. Put Marimo notebook `.py` files there, or override the volume in Compose for another notebook directory.
