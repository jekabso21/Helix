import math

RPM_TO_RAD_PER_S = 2.0 * math.pi / 60.0


def mm_to_m(value: float) -> float:
    return value * 1e-3


def g_to_kg(value: float) -> float:
    return value * 1e-3


def deg_to_rad(value: float) -> float:
    return math.radians(value)


def c_to_k(value: float) -> float:
    return value + 273.15


def hpa_to_pa(value: float) -> float:
    return value * 100.0


def rpm_to_rad_per_s(value: float) -> float:
    return value * RPM_TO_RAD_PER_S
