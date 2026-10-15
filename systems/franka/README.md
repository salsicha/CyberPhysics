# Franka sparse-pose policy simulations

`config/peract_reach.json` and `config/act3d_reach.json` configure the language-conditioned reaching
task for Isaac Sim's Franka Panda. Coordinates are world-frame metres, with
the arm base at the origin and blocks on the ground plane. The reaching
height is 0.20 m; the gripper stays open.

`config/joint_control.json` sets the physical position-drive gains and
minimum-jerk trajectory timing. These executor settings can be tuned without
changing the policy's perception/workspace contract.

The two commands are `reach above the red block` and
`reach above the blue block`. Scenes randomize both block positions; Act3D's
evaluation asks for both colors in each scene before sampling the next scene.
The neural policy must select the requested block from a rendered RGB-D view.

Run, training, architecture and limitations:
[PerAct demo](../../applications/peract/README.md) and
[Act3D demo](../../applications/act3d/README.md).
The default is simulation only; no physical Franka driver is included.
