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

## Fingertip contact regions

Fingertip collisions use upstream revision `46c0b7447edc8254e8b44dd669df86fb6985d876`,
packed into the existing `usd/payloads/geometries.usd`. The 1 mm inner pads use
static/dynamic friction `1.5/1.5`; outer shells use `0.3/0.2`, with zero restitution.
The collision envelope and gripper opening are preserved.

## LICENSE

This software is licensed under the Apache License 2.0. See `LICENSE` for details.
