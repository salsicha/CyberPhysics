# Applications

`applications/` contains reusable Docker images. Each application should be able to run without knowing a specific robot; robot-specific files should be mounted from `systems/` or configured through launch arguments, environment variables, or compose files.

## Build

From the repository root:

```bash
make -C applications build_<app_name>
```

Examples:

```bash
make -C applications build_ros2
make -C applications build_racecarneo
make -C applications build_depthanything
make -C applications build_nicegui
```

`make -C applications help` lists every build target. Each target builds its base image first (for example `build_nicegui` builds `ros2`). Every app is a tag of the Docker Hub repository `salsicha/cyberphysics`: amd64 builds are tagged `<app>-latest` (e.g. `salsicha/cyberphysics:ros2-latest`).

For ARM64 targets (images are tagged `<app>-arm64`):

```bash
cd applications
make target_arm64 build_ros2
```

Builds do not modify the working tree: `.gitattributes` keeps scripts and Dockerfiles at LF line endings in every checkout. `make -C applications cleanup` strips Windows line endings from sources under `applications/` if a copy bypassed git.

## Add An Application

1. Create `applications/<app_name>/` using lowercase words with underscores only when needed.
2. Add a `Dockerfile` and an `entrypoint.sh` if the app needs one.
3. Add a `build_<app_name>` target to `applications/Makefile`.
4. Add a short `README.md` describing what the app provides, how to build it, and the topics/ports/assets it uses.
5. Add or update a compose file in `compositions/` if the app is part of a runnable stack.

A minimal Makefile target looks like this:

```make
build_<app_name>: build_ros2
	APP=<app_name> ./_build_it.sh
```

Use the closest existing Dockerfile as the template. ROS 2 Jazzy apps should normally inherit from `salsicha/cyberphysics:ros2-${TAG}` unless they need a specialized base image. The build script passes `TAG` (`latest` or `arm64`), so declare it before `FROM` with a default that matches locally built images:

```dockerfile
ARG TAG=latest
FROM salsicha/cyberphysics:ros2-${TAG} AS build
```

## Run A Container Directly

```bash
docker run -it --rm salsicha/cyberphysics:ros2-latest /bin/bash
```

## Run With Compose

Compose files live in `compositions/`:

```bash
docker compose -f compositions/racecarneo.yaml up
./start.sh racecarneo    # same thing, from the repository root
```

Use `down --remove-orphans` when changing service sets:

```bash
docker compose -f compositions/racecarneo.yaml down --remove-orphans
```

## Image Hygiene

- Download third-party assets during image build or at runtime; do not commit them.
- Keep generated maps, bags, logs, and model caches out of git.
- Prefer `--no-install-recommends` for apt installs.
- Keep final images runnable headlessly whenever practical.
