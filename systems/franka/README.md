# Franka PerAct simulation

`config/peract_reach.json` configures the basic language-conditioned reaching
task for Isaac Sim's Franka Panda. Coordinates are world-frame metres, with
the arm base at the origin and blocks on the ground plane. The reaching
height is 0.20 m; the gripper stays open.

`config/joint_control.json` sets the physical position-drive gains and
minimum-jerk trajectory timing. These executor settings can be tuned without
changing the policy's perception/workspace contract.

The two commands are `reach above the red block` and
`reach above the blue block`. Each episode randomizes both block positions.
The neural policy must select the requested block from a rendered RGB-D view.

Run, training, architecture and limitations:
[PerAct demo](../../applications/peract/README.md).
The default is simulation only; no physical Franka driver is included.
