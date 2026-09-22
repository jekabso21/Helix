# pyright: reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportAttributeAccessIssue=false, reportOptionalMemberAccess=false
from typing import Any, cast

import numpy as np
import trimesh

from simtools.modelc.compile import CompiledDrone
from simtools.modelc.frames import GLTF_FROM_FRD
from simtools.modelc.mass import Matrix, Vector
from simtools.modelc.parts import PlacedPart

NOSE_COLOR = "#e02020"
NOSE_SIZE_M = (0.02, 0.012, 0.008)


def _rgba(color: str) -> list[int]:
    return [int(color[i : i + 2], 16) for i in (1, 3, 5)] + [255]


def _transform(position_frd: Vector, rotation_frd_from_part: Matrix) -> Any:
    t = np.eye(4)
    t[:3, :3] = GLTF_FROM_FRD @ rotation_frd_from_part
    t[:3, 3] = GLTF_FROM_FRD @ position_frd
    return t


def _mesh(part: PlacedPart) -> Any:
    if part.shape == "box":
        return trimesh.creation.box(extents=part.dims_m)
    if part.shape == "cylinder":
        return trimesh.creation.cylinder(radius=part.dims_m[0], height=part.dims_m[1], sections=24)
    return trimesh.creation.icosphere(subdivisions=2, radius=part.dims_m[0])


def build_scene(model: CompiledDrone) -> Any:
    """Blocky model with the CG at the scene origin and a red nose marker on the front."""
    scene = trimesh.Scene()
    cg = model.cg_from_origin_frd_m
    front = 0.0
    for part in model.parts:
        mesh = _mesh(part)
        mesh.visual.face_colors = _rgba(part.color)
        scene.add_geometry(
            mesh,
            node_name=part.name,
            geom_name=part.name,
            transform=_transform(part.position_frd_m - cg, part.rotation_frd_from_part),
        )
        front = max(front, float(part.position_frd_m[0] - cg[0]) + max(part.dims_m))
    nose = trimesh.creation.box(extents=NOSE_SIZE_M)
    nose.visual.face_colors = _rgba(NOSE_COLOR)
    scene.add_geometry(
        nose,
        node_name="nose_marker",
        geom_name="nose_marker",
        transform=_transform(np.array([front + NOSE_SIZE_M[0] / 2.0, 0.0, 0.0]), np.eye(3)),
    )
    return scene


def export_glb(model: CompiledDrone) -> bytes:
    return cast(bytes, build_scene(model).export(file_type="glb"))
