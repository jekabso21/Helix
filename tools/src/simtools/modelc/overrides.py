from typing import Any

from pydantic import ValidationError

from simtools.config.schemas import DroneConfig
from simtools.modelc.compile import CompiledDrone


class OverrideError(ValueError):
    pass


def apply_overrides(drone: DroneConfig, overrides: dict[str, Any]) -> DroneConfig:
    """Overrides: {"parts": {name: {"pos_mm": [x, y, z], "mass_g": m, "rot_deg": [r, p, y]}}}."""
    data = drone.model_dump()
    parts: dict[str, Any] = data["parts"]
    for name, fields in dict(overrides.get("parts", {})).items():
        if name not in parts:
            raise OverrideError(f"unknown part '{name}'")
        for key, value in dict(fields).items():
            if key not in ("pos_mm", "mass_g", "rot_deg"):
                raise OverrideError(f"part '{name}': '{key}' is not editable")
            parts[name][key] = value
    try:
        return DroneConfig.model_validate(data)
    except ValidationError as error:
        raise OverrideError(str(error)) from error


def summary(model: CompiledDrone) -> dict[str, Any]:
    """What the app's Drone tab shows: editable parts in config units plus the compiled numbers."""
    max_thrust = model.max_thrust_per_motor_n()
    return {
        "name": model.name,
        "mass_kg": model.mass.mass_kg,
        "cg_from_origin_frd_m": [float(v) for v in model.cg_from_origin_frd_m],
        "inertia_frd_kg_m2": [[float(v) for v in row] for row in model.mass.inertia_about_cg],
        "hover": [
            {
                "bf_index": m.bf_index,
                "thrust_n": thrust,
                "fraction": thrust / max_thrust,
                "position_frd_m": [float(v) for v in m.position_frd_m],
            }
            for m, thrust in zip(model.motors, model.static_hover_thrust_n(), strict=True)
        ],
        "max_thrust_n": max_thrust,
        "parts": [
            {
                "name": name,
                "shape": part.shape,
                "mass_g": part.mass_g,
                "pos_mm": list(part.pos_mm),
                "rot_deg": list(part.rot_deg),
            }
            for name, part in model.config.parts.items()
        ],
        "generated_mass_kg": sum(p.mass_kg for p in model.parts if p.generated),
    }
