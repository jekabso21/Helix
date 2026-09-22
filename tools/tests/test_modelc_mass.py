import math

import numpy as np
import pytest

from simtools.modelc import (
    Body,
    box_inertia,
    combine,
    cylinder_inertia,
    rotation_frd_from_part,
    sphere_inertia,
)


def body(
    mass: float, pos: tuple[float, float, float], inertia: np.ndarray, rot: np.ndarray | None = None
) -> Body:
    return Body(mass, np.array(pos, dtype=float), inertia, np.eye(3) if rot is None else rot)


def test_primitive_formulas_match_hand_calculation() -> None:
    assert np.allclose(
        np.diag(box_inertia(2.0, (0.1, 0.2, 0.3))), [2 / 12 * 0.13, 2 / 12 * 0.10, 2 / 12 * 0.05]
    )
    cyl = cylinder_inertia(1.0, 0.1, 0.4)
    assert cyl[2, 2] == pytest.approx(0.005)
    assert cyl[0, 0] == pytest.approx((3 * 0.01 + 0.16) / 12)
    assert np.allclose(sphere_inertia(5.0, 0.2), np.eye(3) * 0.08)


def test_single_box_combines_to_itself() -> None:
    inertia = box_inertia(0.5, (0.15, 0.04, 0.03))
    props = combine([body(0.5, (0.0, 0.0, -0.012), inertia)])
    assert props.mass_kg == 0.5
    assert np.allclose(props.cg_frd_m, [0.0, 0.0, -0.012])
    assert np.allclose(props.inertia_about_cg, inertia)


def test_two_point_masses_give_parallel_axis_result() -> None:
    # two 1 kg points 0.2 m apart along x: I_yy = I_zz = 2 * 1 * 0.1^2, I_xx = 0
    props = combine(
        [
            body(1.0, (0.1, 0.0, 0.0), np.zeros((3, 3))),
            body(1.0, (-0.1, 0.0, 0.0), np.zeros((3, 3))),
        ]
    )
    assert np.allclose(props.cg_frd_m, [0.0, 0.0, 0.0])
    assert np.allclose(np.diag(props.inertia_about_cg), [0.0, 0.02, 0.02])
    assert np.allclose(props.inertia_about_cg - np.diag(np.diag(props.inertia_about_cg)), 0.0)


def test_rotated_cylinder_swaps_axes() -> None:
    # cylinder axis rotated from z to x by a 90 deg pitch: Izz(local) becomes Ixx(body)
    rot = rotation_frd_from_part(0.0, math.pi / 2, 0.0)
    props = combine([body(1.0, (0.0, 0.0, 0.0), cylinder_inertia(1.0, 0.1, 0.4), rot)])
    assert props.inertia_about_cg[0, 0] == pytest.approx(0.005)
    assert props.inertia_about_cg[2, 2] == pytest.approx((3 * 0.01 + 0.16) / 12)


def test_offset_and_rotated_composite_matches_hand_calculation() -> None:
    # 0.3 kg yawed box 50 mm forward, 0.1 kg sphere 100 mm behind and 20 mm up
    box = body(
        0.3,
        (0.05, 0.0, 0.0),
        box_inertia(0.3, (0.04, 0.04, 0.02)),
        rotation_frd_from_part(0.0, 0.0, math.pi / 2),
    )
    ball = body(0.1, (-0.10, 0.0, -0.02), sphere_inertia(0.1, 0.01))
    props = combine([box, ball])
    cg_x = (0.3 * 0.05 + 0.1 * -0.10) / 0.4
    cg_z = (0.1 * -0.02) / 0.4
    assert np.allclose(props.cg_frd_m, [cg_x, 0.0, cg_z])
    dx_box, dz_box = 0.05 - cg_x, 0.0 - cg_z
    dx_ball, dz_ball = -0.10 - cg_x, -0.02 - cg_z
    expected_yy = (
        0.3 * (0.04**2 + 0.02**2) / 12
        + 0.3 * (dx_box**2 + dz_box**2)
        + 0.1 * 2 * 0.01**2 / 5
        + 0.1 * (dx_ball**2 + dz_ball**2)
    )
    assert props.inertia_about_cg[1, 1] == pytest.approx(expected_yy)
    assert props.inertia_about_cg[0, 2] == pytest.approx(
        -(0.3 * dx_box * dz_box + 0.1 * dx_ball * dz_ball)
    )
    assert np.allclose(props.inertia_about_cg, props.inertia_about_cg.T)


def test_rotation_order_is_yaw_pitch_roll() -> None:
    r = rotation_frd_from_part(0.1, 0.2, 0.3)
    assert np.allclose(r @ r.T, np.eye(3))
    assert np.linalg.det(r) == pytest.approx(1.0)
    # pure yaw of 90 deg maps local x to body y
    assert np.allclose(
        rotation_frd_from_part(0.0, 0.0, math.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12
    )
    # pure positive pitch (nose up) maps local x towards -z (up in FRD)
    assert np.allclose(
        rotation_frd_from_part(0.0, math.pi / 2, 0.0) @ [1, 0, 0], [0, 0, -1], atol=1e-12
    )


def test_rejects_empty_or_massless() -> None:
    with pytest.raises(ValueError):
        combine([])
    with pytest.raises(ValueError):
        combine([body(0.0, (0, 0, 0), np.zeros((3, 3)))])
