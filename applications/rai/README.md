
# RAI

This app serves as a demonstration robotic agents.

See original project:
https://github.com/RobotecAI/rai

The image pins a RAI release (`RAI_REF` build argument), installs its Python
environment with `uv` into `/rai/.venv`, and builds the RAI ROS 2 (Humble)
workspace in `/rai`. By default the container serves the RAI configurator on
`http://localhost:8501`:

```bash
make -C applications build_rai
docker compose -f compositions/rai.yaml up
```
