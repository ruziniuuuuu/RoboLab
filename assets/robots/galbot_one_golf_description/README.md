# Galbot One Golf Description

USD description of the Galbot One Golf robot, used by the RoboLab Galbot Golf
embodiment (`robolab/robots/galbot_golf.py`).

## Checkout

This package was imported from the official Galbot description repository:

- Source: `https://github.com/GalaxyGeneralRobotics/galbot_one_golf_description`
- Revision: `b311f5ca1acf506e9b7026397e2c74fb2db11df6`

RoboLab retains only the USD assets, which are self-contained (meshes are
embedded in `usd/payloads/geometries.usd`; textures live under
`usd/Textures/`). The upstream URDF, MJCF, xacro, mesh, rviz, and launch
files are not used by RoboLab and were dropped. Use the source checkout at
the revision above when you need them (e.g. URDF for planning tools, or
regenerating descriptions from xacro).

## USD

The main USD entry point is `usd/galbot_one_golf.usda`. Related payloads and
textures are stored under `usd/`.

Variant sets on the root prim (defaults in bold): Physics (**physx**,
physics, none, mujoco), Robot (**none**, robot), Sensor (**none**, sensors).

### Wrist camera visual and calibration

The existing `d405` visual prims use the forward `d405_sz073` assembly from
the description repository at revision `69cdfde0178f2eed10c4b2e612c0c3d3b2263c7a`
(`usd/components/camera/camera_d405_sz073/payloads/instances.usda`,
`/Instances/mesh`). Its visual meshes and materials are embedded in the existing
`geometries.usd` entry `/Geometries/mesh_49`. The assembly includes its stand,
so the original `d405_stand` visuals are hidden. Link names, transforms,
collision geometry, and physical properties are unchanged.

RoboLab's `left_wrist_cam` and `right_wrist_cam` use SynthNova's Yundonghui
policy calibration (`extensions/yundonghui/src/synthnova_yundonghui/cameras.py`,
SynthNova revision `429f5a4`). Images are resized from 400 x 224 to 640 x 360,
scaling each intrinsic-matrix row by its corresponding image dimension:
`fx=323.2`, `fy=324.64285714285717`, `cx=320`, `cy=180`.
The wrist sensor applies Isaac Sim's OpenCV pinhole lens schema (zero distortion)
and reports the same matrix, preserving the unequal focal lengths that the
default RTX pinhole and IsaacLab square-pixel assumption would otherwise lose.
The source ROS optical extrinsics are relative to each
`arm_end_effector_mount_link`. The camera config composes that mount's fixed
transform into `arm_link7` and converts XYZW quaternions to WXYZ, allowing
the sensors to follow the moving rigid bodies directly.

## LICENSE

This software is licensed under the Apache License 2.0. See `LICENSE` for details.
