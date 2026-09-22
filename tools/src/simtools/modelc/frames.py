import numpy as np

from simtools.modelc.mass import Matrix, Vector

# x_gltf = -R (left), y_gltf = -D (up), z_gltf = F (forward); the only FRD to glTF mapping in tools
GLTF_FROM_FRD: Matrix = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def gltf_from_frd(v: Vector) -> Vector:
    return GLTF_FROM_FRD @ v


def gltf_rotation_from_frd(r_frd: Matrix) -> Matrix:
    return GLTF_FROM_FRD @ r_frd @ GLTF_FROM_FRD.T


def quaternion_wxyz_from_rotation(r: Matrix) -> tuple[float, float, float, float]:
    """Shepperd's method; returns (w, x, y, z) with w >= 0."""
    trace = float(np.trace(r))
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w, x, y, z = (
            0.25 * s,
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
        )
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w, x, y, z = (
            (r[2, 1] - r[1, 2]) / s,
            0.25 * s,
            (r[0, 1] + r[1, 0]) / s,
            (r[0, 2] + r[2, 0]) / s,
        )
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w, x, y, z = (
            (r[0, 2] - r[2, 0]) / s,
            (r[0, 1] + r[1, 0]) / s,
            0.25 * s,
            (r[1, 2] + r[2, 1]) / s,
        )
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w, x, y, z = (
            (r[1, 0] - r[0, 1]) / s,
            (r[0, 2] + r[2, 0]) / s,
            (r[1, 2] + r[2, 1]) / s,
            0.25 * s,
        )
    sign = -1.0 if w < 0.0 else 1.0
    return (float(sign * w), float(sign * x), float(sign * y), float(sign * z))
