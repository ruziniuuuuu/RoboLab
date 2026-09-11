# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Runtime projection contract for resized Galbot wrist observations."""

import isaaclab.sim as sim_utils
import pytest
from isaaclab.sim import SimulationCfg, SimulationContext
from isaacsim.core.utils.stage import create_new_stage, get_current_stage

from robolab.robots.galbot_golf import GalbotGolfLeftWristCameraCfg


def test_wrist_renderer_and_observation_preserve_independent_focal_lengths():
    create_new_stage()
    sim = SimulationContext(SimulationCfg(dt=0.01, device="cuda:0"))
    camera = None
    try:
        stage = get_current_stage()
        stage.DefinePrim("/World/wrist", "Xform")
        light = sim_utils.DomeLightCfg(intensity=1000.0)
        light.func("/World/light", light)
        box = sim_utils.CuboidCfg(size=(0.1, 0.1, 0.1))
        box.func("/World/box", box)
        cfg = GalbotGolfLeftWristCameraCfg().left_wrist_cam.replace(
            prim_path="/World/wrist/left_wrist_cam"
        )
        camera = cfg.class_type(cfg)
        sim.reset()
        for _ in range(3):
            sim.step()
            camera.update(sim.get_physics_dt())
        assert tuple(camera.data.output["rgb"].shape) == (1, 360, 640, 3)
        matrix = camera.data.intrinsic_matrices[0].cpu().numpy()
        assert matrix[0].tolist() == pytest.approx([323.2, 0.0, 320.0])
        assert matrix[1].tolist() == pytest.approx([0.0, 324.64285714285717, 180.0])
        assert matrix[2].tolist() == pytest.approx([0.0, 0.0, 1.0])
        prim = stage.GetPrimAtPath(cfg.prim_path)
        assert prim.GetAttribute("omni:lensdistortion:model").Get() == "opencvPinhole"
        for name, value in (("fx", matrix[0, 0]), ("fy", matrix[1, 1])):
            assert prim.GetAttribute(f"omni:lensdistortion:opencvPinhole:{name}").Get() == pytest.approx(value)
    finally:
        if camera is not None:
            del camera
        sim.clear_all_callbacks()
        sim.clear_instance()
        sim.stop()
