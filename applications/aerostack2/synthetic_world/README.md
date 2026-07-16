# synthetic_world

Single source of truth for the procedural terrain and satellite imagery used
to validate the vision-navigation stack (demnav, wildnav) in simulation, plus
the shared local-tangent georeferencing helpers.

Installed into `/venv` by the `cyberphysics/aerostack2` Dockerfile, so it is
importable in every derived image (`cyberphysics/demnav`,
`cyberphysics/wildnav`) and by scripts run directly in the base image (e.g.
`systems/airplane/scripts/satellite_camera_sim.py`).

Do not copy these functions into other modules: cameras render frames that
must match the reference tiles/DEM bit-for-bit, and any drift between copies
silently degrades matching.
