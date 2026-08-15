# Runtime data

This directory provides host-side persistent storage for application data that
is mounted into Docker containers. Keep generated models, checkpoints, and
telemetry out of source files and place them in the appropriate application
subdirectory instead.

The SmolVLA layout is:

- `smolvla/models/` for downloaded or exported model checkpoints;
- `smolvla/training/` for fine-tuning runs and training checkpoints; and
- `smolvla/telemetry/` for SO101 simulation telemetry.

The placeholder files keep the empty directory structure in Git. Large runtime
artifacts should not be committed.
