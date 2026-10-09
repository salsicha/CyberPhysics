# Unreal Engine server

`run_server.sh` runs a packaged UE Linux build headless on the GPU. Put the
packaged folder (the one with `<Project>.sh`) in `build/`, which git and the
Docker build ignore, or mount it at `/server/build` yourself. Settings are
environment variables documented at the top of `run_server.sh` and in
`../README.md`.
