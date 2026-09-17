"""SE(3) helpers: 4x4 homogeneous transforms, exp/log, and quaternion averaging."""

import numpy as np

_EPS = 1e-9


def hat(w):
    """Skew-symmetric matrix of a 3-vector."""
    wx, wy, wz = w
    return np.array([[0.0, -wz, wy], [wz, 0.0, -wx], [-wy, wx, 0.0]])


def quat_to_matrix(qw, qx, qy, qz):
    """Rotation matrix from a (w, x, y, z) quaternion."""
    n = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < _EPS:
        return np.eye(3)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )


def matrix_to_quat(r):
    """(w, x, y, z) quaternion from a rotation matrix (Shepperd's method)."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        return np.array(
            [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
        )
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = np.empty(4)
    q[0] = (r[k, j] - r[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (r[j, i] + r[i, j]) / s
    q[1 + k] = (r[k, i] + r[i, k]) / s
    return q


def pose_to_matrix(pose7):
    """(x, y, z, qw, qx, qy, qz) -- the WPILib Pose3d struct layout -- to a 4x4."""
    x, y, z, qw, qx, qy, qz = pose7
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(qw, qx, qy, qz)
    t[:3, 3] = (x, y, z)
    return t


def make(translation, rotation_matrix):
    t = np.eye(4)
    t[:3, :3] = rotation_matrix
    t[:3, 3] = translation
    return t


def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def inv(t):
    """Inverse of a 4x4 rigid transform (transpose, not a general inverse)."""
    out = np.eye(4)
    rt = t[:3, :3].T
    out[:3, :3] = rt
    out[:3, 3] = -rt @ t[:3, 3]
    return out


def so3_exp(w):
    theta = np.linalg.norm(w)
    wx = hat(w)
    if theta < 1e-8:
        return np.eye(3) + wx + 0.5 * wx @ wx
    return (
        np.eye(3)
        + (np.sin(theta) / theta) * wx
        + ((1 - np.cos(theta)) / (theta * theta)) * wx @ wx
    )


def so3_log(r):
    cos_theta = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-8:
        # Near identity the antisymmetric part is already the rotation vector.
        return np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) * 0.5
    if np.pi - theta < 1e-6:
        # Near pi the antisymmetric part degenerates; recover the axis from the quaternion.
        q = matrix_to_quat(r)
        axis = q[1:]
        n = np.linalg.norm(axis)
        return axis / n * theta if n > _EPS else np.zeros(3)
    factor = theta / (2.0 * np.sin(theta))
    return factor * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def _left_jacobian(w):
    theta = np.linalg.norm(w)
    wx = hat(w)
    if theta < 1e-8:
        return np.eye(3) + 0.5 * wx + (1.0 / 6.0) * wx @ wx
    return (
        np.eye(3)
        + ((1 - np.cos(theta)) / theta**2) * wx
        + ((theta - np.sin(theta)) / theta**3) * wx @ wx
    )


def exp(xi):
    """se(3) -> SE(3). xi is (translation 3, rotation 3)."""
    rho, w = xi[:3], xi[3:]
    t = np.eye(4)
    t[:3, :3] = so3_exp(w)
    t[:3, 3] = _left_jacobian(w) @ rho
    return t


def log(t):
    """SE(3) -> se(3), returning (translation 3, rotation 3)."""
    w = so3_log(t[:3, :3])
    rho = np.linalg.solve(_left_jacobian(w), t[:3, 3])
    return np.concatenate([rho, w])


def average_poses(matrices):
    """Mean of several 4x4 transforms.

    Translations are averaged arithmetically; rotations via the dominant eigenvector
    of the summed quaternion outer products, which is the standard chordal-L2 mean
    and is sign-ambiguity free.
    """
    matrices = list(matrices)
    translation = np.mean([m[:3, 3] for m in matrices], axis=0)
    acc = np.zeros((4, 4))
    for m in matrices:
        q = matrix_to_quat(m[:3, :3])
        acc += np.outer(q, q)
    _, vecs = np.linalg.eigh(acc)
    q = vecs[:, -1]
    return make(translation, quat_to_matrix(*q))


def rpy_degrees(r):
    """WPILib Rotation3d roll/pitch/yaw (intrinsic Z-Y-X) in degrees."""
    pitch = np.arcsin(np.clip(-r[2, 0], -1.0, 1.0))
    if abs(r[2, 0]) > 1 - 1e-9:  # gimbal lock
        roll = np.arctan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    else:
        roll = np.arctan2(r[2, 1], r[2, 2])
        yaw = np.arctan2(r[1, 0], r[0, 0])
    return np.degrees([roll, pitch, yaw])
