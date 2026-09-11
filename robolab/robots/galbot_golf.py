# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Fixed-base ``galbot_one_golf`` dual-arm robot configuration."""

import os
from collections.abc import Sequence

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sensors import TiledCamera, TiledCameraCfg
from isaaclab.utils import configclass
from pxr import Gf, Sdf

from robolab.constants import ROBOTS_DIR
from robolab.robots.galbot_golf_definitions import *  # noqa

GALBOT_GOLF_ASSET_DIR = os.path.join(ROBOTS_DIR, "galbot_one_golf_description")
GALBOT_GOLF_USD_PATH = os.environ.get(
    "GALBOT_GOLF_USD_PATH",
    os.path.join(GALBOT_GOLF_ASSET_DIR, "usd", "galbot_one_golf.usda"),
)

GALBOT_GOLF_USD_VARIANTS = {
    "Physics": "physx",
    "Robot": "robot",
    "Sensor": "sensors",
}

# The Golf's lowest colliders — the passive caster-wheel spheres — reach
# 26.4 mm below the root origin (measured by check_standing_contact.py's
# static scan). The env factory places the root this far above each scene's
# authored /GroundPlane via the root_z_above_ground label, so the wheels rest
# exactly on the floor at any scene ground height: -0.6706 in canonical
# -0.697 scenes, -0.6236 in legacy -0.65 scenes (tests/test_scene_ground.py
# locks the per-scene heights). The root is welded (fix_root_link), so the
# robot needs no ground support either way.
GALBOT_GOLF_ROOT_Z_ABOVE_GROUND = 0.0264
# Canonical-scene default, for instantiating the cfg outside the env factory.
GALBOT_GOLF_TABLETOP_ROOT_POS = (0.0, 0.0, -0.697 + GALBOT_GOLF_ROOT_Z_ABOVE_GROUND)


########################################################
# Robot-attached and replay cameras
########################################################


def _source_replay_camera(
    name: str,
    *,
    parent_path: str = "base_footprint/base_link",
    pos: tuple[float, float, float],
    rot: tuple[float, float, float, float],
    focal_length: float,
    focus_distance: float,
    horizontal_aperture: float,
) -> TiledCameraCfg:
    """Robot-attached camera matching the existing Galbot replay/test camera convention."""
    return TiledCameraCfg(
        prim_path=f"{{ENV_REGEX_NS}}/robot/{parent_path}/{name}",
        height=720,
        width=1280,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=focal_length,
            focus_distance=focus_distance,
            horizontal_aperture=horizontal_aperture,
            clipping_range=(0.1, 1.0e5),
        ),
        offset=TiledCameraCfg.OffsetCfg(pos=pos, rot=rot, convention="opengl"),
    )


class _WristCamera(TiledCamera):
    """Render and report the independently resized wrist focal lengths."""

    def __init__(self, cfg: TiledCameraCfg) -> None:
        super().__init__(cfg)
        # RTX's default pinhole and IsaacLab's intrinsic reporting assume
        # square pixels. OpenCV pinhole preserves fx != fy after resizing.
        for prim in sim_utils.find_matching_prims(cfg.prim_path):
            # Author the schema and attributes explicitly: headless Kit apps
            # need not have registered the optional schema's USD fallbacks.
            prim.AddAppliedSchema("OmniLensDistortionOpenCvPinholeAPI")
            prim.CreateAttribute("omni:lensdistortion:model", Sdf.ValueTypeNames.Token).Set("opencvPinhole")
            parameters = {
                "fx": cfg.width * cfg.spawn.focal_length / cfg.spawn.horizontal_aperture,
                "fy": cfg.height * cfg.spawn.focal_length / cfg.spawn.vertical_aperture,
                "cx": cfg.width / 2,
                "cy": cfg.height / 2,
                **dict.fromkeys(("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2", "s1", "s2", "s3", "s4"), 0.0),
            }
            for name, value in parameters.items():
                prim.CreateAttribute(f"omni:lensdistortion:opencvPinhole:{name}", Sdf.ValueTypeNames.Float).Set(value)
            prim.CreateAttribute("omni:lensdistortion:opencvPinhole:imageSize", Sdf.ValueTypeNames.Int2).Set(
                Gf.Vec2i(cfg.width, cfg.height)
            )

    def _update_intrinsic_matrices(self, env_ids: Sequence[int]) -> None:
        super()._update_intrinsic_matrices(env_ids)
        for index in env_ids:
            prim = self._sensor_prims[index].GetPrim()
            self._data.intrinsic_matrices[index, 1, 1] = prim.GetAttribute(
                "omni:lensdistortion:opencvPinhole:fy"
            ).Get()


def _wrist_camera(side: str) -> TiledCameraCfg:
    """Yundonghui policy calibration resized from 400x224 to 640x360."""
    # SynthNova extensions/yundonghui/src/synthnova_yundonghui/cameras.py:
    # ROS optical pose relative to the arm end-effector mount, quaternion XYZW.
    mount_pos = (0.07089459385344977, 0.011084636915734618, 0.0475356786953811)
    mount_quat = (-0.5921350168801781, 0.5860286987303605, -0.38397145780165504, 0.3981361647004496)
    if side == "right":
        mount_quat = tuple(-value for value in mount_quat)
    x, y, z, w = mount_quat
    # arm_link7 -> mount: translation (-0.10926, 0, 0), rotation Ry(pi).
    # Attach directly to the moving rigid body so Fabric updates the camera;
    # compose the mount transform and convert XYZW to IsaacLab's WXYZ order.
    return TiledCameraCfg(
        class_type=_WristCamera,
        prim_path=f"{{ENV_REGEX_NS}}/robot/{side}_arm_link7/{side}_wrist_cam",
        height=360,
        width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            # Keep the source film gate: fx=202*640/400, fy=202*360/224,
            # with centered principal point (320, 180) after resizing.
            focal_length=202 * 0.03,
            focus_distance=0.0,
            horizontal_aperture=400 * 0.03,
            vertical_aperture=224 * 0.03,
            clipping_range=(0.03, 10.0),
        ),
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.10926 - mount_pos[0], mount_pos[1], -mount_pos[2]),
            rot=(-y, z, w, -x),
            convention="ros",
        ),
    )


_LEFT_WRIST_CAM = _wrist_camera("left")
_RIGHT_WRIST_CAM = _wrist_camera("right")
_FRONT_CAM = _source_replay_camera(
    "front_camera",
    pos=(2.016, 0.0, 1.826),
    rot=(0.603996, 0.367682, 0.367682, 0.603996),
    focal_length=16.0,
    focus_distance=150.0,
    horizontal_aperture=20.955,
)
_LEFT_EGO_CAM = TiledCameraCfg(
    prim_path=(
        "{ENV_REGEX_NS}/robot/head_link2/head_end_effector_mount_link/"
        "camera_front_head_left_rgb"
    ),
    height=480,
    width=640,
    data_types=["rgb"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=0.488472287905,
        horizontal_aperture=1.280137468212909,
        vertical_aperture=0.9598969209810818,
        clipping_range=(0.08, 15.0),
    ),
    offset=TiledCameraCfg.OffsetCfg(
        pos=(0.07026692, -0.05334402, 0.02098859),
        rot=(0.69873623, -0.09626416, 0.70197894, 0.09862283),
        convention="ros",
    ),
)


def _galbot_golf_robot_cfg(
    *,
    init_joint_pos: dict[str, float] | None = None,
    init_pos: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> ArticulationCfg:
    """Build a Golf articulation config while preserving the received USD physics properties."""
    return ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=GALBOT_GOLF_USD_PATH,
            variants=GALBOT_GOLF_USD_VARIANTS,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                fix_root_link=True,
                solver_position_iteration_count=128,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=init_pos,
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos=init_joint_pos
            or {
                "leg_joint.*": 0.0,
                "head_joint.*": 0.0,
                "left_arm_joint.*": 0.0,
                "right_arm_joint.*": 0.0,
                "left_gripper_joint": GRIPPER_OPEN,
                "right_gripper_joint": GRIPPER_OPEN,
                ".*_gripper_.*_joint": 0.0,
                "wheel.*joint": 0.0,
            },
        ),
        soft_joint_pos_limit_factor=1.0,
        actuators={
            # Preserve all drive properties authored in the supplied PhysX USD.
            "legs": ImplicitActuatorCfg(
                joint_names_expr=["leg_joint.*"],
                effort_limit_sim=None,
                velocity_limit_sim=None,
                stiffness=None,
                damping=None,
            ),
            "head": ImplicitActuatorCfg(
                joint_names_expr=["head_joint.*"],
                effort_limit_sim=None,
                velocity_limit_sim=None,
                stiffness=None,
                damping=None,
            ),
            "arms": ImplicitActuatorCfg(
                joint_names_expr=["left_arm_joint.*", "right_arm_joint.*"],
                effort_limit_sim=None,
                velocity_limit_sim=None,
                stiffness=None,
                damping=None,
            ),
            "grippers": ImplicitActuatorCfg(
                joint_names_expr=["left_gripper_joint", "right_gripper_joint"],
                effort_limit_sim=1.0,
                velocity_limit_sim=3.5,
                stiffness=40.0,
                damping=5.0,
            ),
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=WHEEL_JOINTS,
                effort_limit_sim=None,
                velocity_limit_sim=None,
                stiffness=None,
                damping=None,
            ),
        },
    )


@configclass
class GalbotGolfFixedBaseCfg:
    """Fixed-root Galbot One Golf articulation for static-base manipulation replay."""

    robot = _galbot_golf_robot_cfg()

    left_wrist_cam = _LEFT_WRIST_CAM
    right_wrist_cam = _RIGHT_WRIST_CAM


@configclass
class GalbotGolfFixedBaseDefaultPoseCfg(GalbotGolfFixedBaseCfg):
    """Fixed-root Golf with a RoboLab reset posture for manipulation tasks."""

    robot = _galbot_golf_robot_cfg(init_joint_pos=GALBOT_GOLF_DEFAULT_JOINT_POS)


@configclass
class GalbotGolfTabletopCfg(GalbotGolfFixedBaseDefaultPoseCfg):
    """Fixed-base Golf placed on the floor of standard RoboLab table scenes."""

    robot = _galbot_golf_robot_cfg(
        init_joint_pos=GALBOT_GOLF_DEFAULT_JOINT_POS,
        init_pos=GALBOT_GOLF_TABLETOP_ROOT_POS,
    )


@configclass
class GalbotGolfReplayCfg(GalbotGolfFixedBaseDefaultPoseCfg):
    """Fixed-base Golf plus report-only front and left-ego cameras."""

    front_cam = _FRONT_CAM
    left_ego_cam = _LEFT_EGO_CAM


@configclass
class GalbotGolfTabletopReplayCfg(GalbotGolfTabletopCfg):
    """Tabletop Golf plus report-only front and left-ego cameras."""

    front_cam = _FRONT_CAM
    left_ego_cam = _LEFT_EGO_CAM


@configclass
class GalbotGolfLeftWristCameraCfg:
    left_wrist_cam = _LEFT_WRIST_CAM


@configclass
class GalbotGolfRightWristCameraCfg:
    right_wrist_cam = _RIGHT_WRIST_CAM


@configclass
class GalbotGolfFrontCameraCfg:
    front_cam = _FRONT_CAM


@configclass
class GalbotGolfLeftEgoCameraCfg:
    left_ego_cam = _LEFT_EGO_CAM


# Class-level labels, assigned after ALL Golf cfg classes: configclass
# decoration of a subclass converts inherited plain attributes into config
# fields, so these must come after the last subclass definition.
# See docs/robots.md#table-fixture.
GalbotGolfFixedBaseCfg.table_fixture = None  # Golf has its own base
# Bimanual: one EE-pose channel per arm. A single-arm channel would be
# meaningless here, and a "base_link" body name would match the Golf's torso
# base instead of either gripper.
GalbotGolfFixedBaseCfg.ee_recorder_bodies = {
    "left_ee_pose": LEFT_EE_BODY,
    "right_ee_pose": RIGHT_EE_BODY,
}
# Floor-standing: the env factory rebases the root onto each scene's authored
# ground. Only the Tabletop cfgs stand on the task-scene floor; the plain
# fixed-base cfgs keep their explicit init pos.
GalbotGolfTabletopCfg.root_z_above_ground = GALBOT_GOLF_ROOT_Z_ABOVE_GROUND
