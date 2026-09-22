import copy
import io
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import trimesh
from pydantic import ValidationError

from simtools.config import load_yaml
from simtools.config.schemas import DroneConfig
from simtools.modelc import compile_drone, export_glb, report, to_json
from simtools.modelc.compile import STANDARD_GRAVITY_MPS2
from simtools.modelc.parts import PlacedPart, rotation_aligning_z_to

REPO_ROOT = Path(__file__).resolve().parents[2]
REFERENCE = REPO_ROOT / "configs/drones/reference_5in.yaml"


def reference_yaml() -> dict[str, Any]:
    data = load_yaml(REFERENCE)
    for key in ("motor", "prop", "battery"):
        data[key] = load_yaml(REPO_ROOT / data[key])
    return data


def minimal(**layout: Any) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "name": "t",
        "layout": {"type": "quad_x", "motor_spacing_mm": 200.0, **layout},
        "motor": {
            "model": "first_order",
            "kv_rpm_per_v": 1900,
            "winding_resistance_ohm": 0.18,
            "rotor_inertia_kg_m2": 1e-6,
            "max_rpm": 20000,
        },
        "prop": {"diameter_mm": 127, "pitch_mm": 109, "k_t": 1e-6, "k_q": 1e-8},
        "battery": {"cells": 6, "capacity_mah": 1000, "cell_resistance_mohm": 8},
        "parts": {"body": {"shape": "box", "size_mm": [100, 40, 20], "mass_g": 300}},
    }


def test_reference_totals_and_symmetry() -> None:
    model = compile_drone(DroneConfig.model_validate(reference_yaml()))
    assert model.mass.mass_kg == pytest.approx(0.497)
    assert model.cg_from_origin_frd_m[0] == pytest.approx(0.0, abs=1e-3)
    assert model.cg_from_origin_frd_m[1] == pytest.approx(0.0, abs=1e-9)
    assert model.cg_from_origin_frd_m[2] < 0.0  # battery and stack sit above the motor plane
    inertia = model.mass.inertia_about_cg
    assert inertia[2, 2] > inertia[0, 0] > 0.0 and inertia[2, 2] > inertia[1, 1] > 0.0
    assert abs(inertia[0, 1]) < 1e-6
    thrusts = model.static_hover_thrust_n()
    assert thrusts == pytest.approx([0.497 * STANDARD_GRAVITY_MPS2 / 4] * 4, rel=1e-2)


def test_moving_the_battery_forward_shifts_the_cg_by_the_lever_rule() -> None:
    base = reference_yaml()
    shifted = copy.deepcopy(base)
    shifted["parts"]["battery"]["pos_mm"][0] += 20.0
    before = compile_drone(DroneConfig.model_validate(base))
    after = compile_drone(DroneConfig.model_validate(shifted))
    expected = 0.185 * 0.020 / before.mass.mass_kg
    delta = after.cg_from_origin_frd_m - before.cg_from_origin_frd_m
    assert delta == pytest.approx([expected, 0.0, 0.0], abs=1e-9)
    # motor positions are relative to the CG, so the front motors move closer
    assert after.motors[1].position_frd_m[0] == pytest.approx(
        before.motors[1].position_frd_m[0] - expected
    )


def test_static_hover_loads_follow_the_moment_balance() -> None:
    shifted = reference_yaml()
    shifted["parts"]["battery"]["pos_mm"][0] += 20.0
    model = compile_drone(DroneConfig.model_validate(shifted))
    a = 0.226 / 2 / math.sqrt(2)
    d = model.cg_from_origin_frd_m[0]
    weight = model.mass.mass_kg * STANDARD_GRAVITY_MPS2
    front, rear = weight * (a + d) / (4 * a), weight * (a - d) / (4 * a)
    thrusts = model.static_hover_thrust_n()
    assert thrusts == pytest.approx([rear, front, rear, front], rel=1e-6)
    assert front > rear


def test_stretched_x_and_custom_layouts() -> None:
    cfg = minimal(type="stretched_x", motor_spacing_mm=None, length_mm=200.0, width_mm=100.0)
    model = compile_drone(DroneConfig.model_validate(cfg))
    origin_positions = [m.position_frd_m + model.cg_from_origin_frd_m for m in model.motors]
    assert origin_positions[1] == pytest.approx([0.1, 0.05, 0.0])
    assert origin_positions[2] == pytest.approx([-0.1, -0.05, 0.0])

    custom = minimal(
        type="custom",
        motor_spacing_mm=None,
        motors=[
            {"bf_index": 2, "pos_mm": [100, 0, 0], "spin": "ccw"},
            {"bf_index": 1, "pos_mm": [-50, 80, 0], "spin": "cw", "axis": [0, 0.1, -1]},
            {"bf_index": 3, "pos_mm": [-50, -80, 0], "spin": "cw"},
        ],
    )
    model = compile_drone(DroneConfig.model_validate(custom))
    assert [m.bf_index for m in model.motors] == [1, 2, 3]
    assert [m.spin for m in model.motors] == [1.0, -1.0, 1.0]
    assert np.linalg.norm(model.motors[0].axis_frd) == pytest.approx(1.0)
    assert {p.name for p in model.parts} >= {"arm1", "motor1", "prop1", "arm3", "prop3", "body"}


def test_mounts_follow_their_part() -> None:
    cfg = minimal()
    cfg["parts"]["cam"] = {
        "shape": "box",
        "size_mm": [20, 20, 20],
        "mass_g": 10,
        "pos_mm": [50, 0, -10],
        "rot_deg": [0, 90, 0],
    }
    cfg["imu"] = {"part": "body", "offset_mm": [5, 0, 0]}
    cfg["cameras"] = [{"name": "fpv", "part": "cam", "offset_mm": [10, 0, 0]}]
    model = compile_drone(DroneConfig.model_validate(cfg))
    cg = model.cg_from_origin_frd_m
    assert model.imu.position_frd_m + cg == pytest.approx([0.005, 0.0, 0.0])
    # a 90 deg nose-up pitch maps the camera's local +x offset to up (-z)
    assert model.cameras[0].position_frd_m + cg == pytest.approx([0.05, 0.0, -0.02], abs=1e-12)
    assert model.cameras[0].rotation_frd_from_mount @ [1, 0, 0] == pytest.approx(
        [0, 0, -1], abs=1e-12
    )


def test_projected_areas_of_primitives() -> None:
    box = PlacedPart("b", "box", (0.1, 0.2, 0.3), 1.0, np.zeros(3), np.eye(3), "#000000", False)
    assert box.projected_areas() == pytest.approx([0.06, 0.03, 0.02])
    cyl = PlacedPart("c", "cylinder", (0.1, 0.4), 1.0, np.zeros(3), np.eye(3), "#000000", False)
    assert cyl.projected_areas() == pytest.approx([0.08, 0.08, math.pi * 0.01])
    tilted = PlacedPart(
        "t",
        "cylinder",
        (0.1, 0.4),
        1.0,
        np.zeros(3),
        rotation_aligning_z_to(np.array([1.0, 0, 0])),
        "#000000",
        False,
    )
    assert tilted.projected_areas() == pytest.approx([math.pi * 0.01, 0.08, 0.08])
    sphere = PlacedPart("s", "sphere", (0.1,), 1.0, np.zeros(3), np.eye(3), "#000000", False)
    assert sphere.projected_areas() == pytest.approx([math.pi * 0.01] * 3)


def test_aero_override_replaces_the_estimate() -> None:
    cfg = minimal()
    estimated = compile_drone(DroneConfig.model_validate(cfg))
    assert estimated.cda_frd_m2[2] > estimated.cda_frd_m2[0] > 0.0  # props and arms are wide, flat
    cfg["aero"] = {"override": {"cda_m2": [0.01, 0.02, 0.03], "cop_mm": [0, 0, -10]}}
    overridden = compile_drone(DroneConfig.model_validate(cfg))
    assert overridden.cda_frd_m2 == pytest.approx([0.01, 0.02, 0.03])
    assert overridden.cop_frd_m == pytest.approx(
        np.array([0, 0, -0.01]) - overridden.cg_from_origin_frd_m
    )


def test_json_and_report_are_complete() -> None:
    model = compile_drone(DroneConfig.model_validate(reference_yaml()))
    data = to_json(model)
    assert set(data) >= {
        "mass_kg",
        "cg_from_origin_frd_m",
        "inertia_frd_kg_m2",
        "motors",
        "imu",
        "cameras",
        "aero",
        "contact",
        "parts",
    }
    assert len(data["contact"]["points_frd_m"]) == 4
    assert data["cameras"][0]["name"] == "main_fpv"
    assert data["imu"]["q_frd_from_imu"] == pytest.approx([1.0, 0.0, 0.0, 0.0])
    names = [p["name"] for p in data["parts"]]
    assert "battery" in names and "motor4" in names
    text = report(model)
    assert "Static hover loads" in text and "| battery |" in text


def test_glb_has_every_part_and_the_nose_marker_points_forward() -> None:
    model = compile_drone(DroneConfig.model_validate(reference_yaml()))
    scene = trimesh.load(io.BytesIO(export_glb(model)), file_type="glb")
    names = set(scene.graph.nodes_geometry)
    assert {"battery", "camera", "arm1", "motor2", "prop3", "nose_marker"} <= names
    nose = scene.graph.get("nose_marker")[0][:3, 3]
    battery = scene.graph.get("battery")[0][:3, 3]
    assert nose[2] > 0.1 and abs(nose[0]) < 1e-9  # forward is glTF +z
    assert battery[1] > 0.0  # above the motor plane is glTF +y
    assert nose[2] > max(scene.graph.get(n)[0][2, 3] for n in names - {"nose_marker"})


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda c: c["parts"].__setitem__("motor1", c["parts"]["body"]), "reserved"),
        (lambda c: c.__setitem__("imu", {"part": "nope"}), "unknown part"),
        (
            lambda c: c["parts"].__setitem__("m", {"shape": "mesh", "file": "a.stl", "mass_g": 1}),
            "not supported",
        ),
        (
            lambda c: c["layout"].update(
                type="custom",
                motor_spacing_mm=None,
                props_out=True,
                motors=[{"bf_index": 1, "pos_mm": [0, 0, 0], "spin": "cw"}],
            ),
            "props_out",
        ),
        (
            lambda c: c["layout"].update(
                type="custom",
                motor_spacing_mm=None,
                motors=[{"bf_index": 2, "pos_mm": [0, 0, 0], "spin": "cw"}],
            ),
            "without gaps",
        ),
    ],
)
def test_invalid_parts_lists_are_rejected(mutate: Any, message: str) -> None:
    cfg = minimal()
    mutate(cfg)
    with pytest.raises(ValidationError, match=message):
        DroneConfig.model_validate(cfg)


def test_golden_compiled_reference_is_current() -> None:
    golden = REPO_ROOT / "tests/golden/config/reference_5in.drone.json"
    current = to_json(compile_drone(DroneConfig.model_validate(reference_yaml())))
    assert json.loads(golden.read_text()) == json.loads(json.dumps(current))
