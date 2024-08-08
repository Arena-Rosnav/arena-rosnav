import collections
import typing
import numpy as np

def quaternion_from_euler(roll: float, pitch: float, yaw: float, **kwargs) -> typing.Tuple[float, float, float, float]:
    """
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code
    """
    cr = np.cos(roll * 0.5);
    sr = np.sin(roll * 0.5);
    cp = np.cos(pitch * 0.5);
    sp = np.sin(pitch * 0.5);
    cy = np.cos(yaw * 0.5);
    sy = np.sin(yaw * 0.5);

    axes: str = str(kwargs.get('axes', 'sxyz'))
    assert len(set(axes)) == len(axes), 'axes contains duplicate entries'
    assert len(set(axes).difference(set('sxyz'))) == 0, 'axes contains invalid entries. allowed: s,x,y,z'

    quat = dict(
        s = cr * cp * cy + sr * sp * sy,
        x = sr * cp * cy - cr * sp * sy,
        y = cr * sp * cy + sr * cp * sy,
        z = cr * cp * sy - sr * sp * cy
    )

    return tuple(quat[axis] for axis in axes)


def euler_from_quaternion(
        x: float,
        y: float,
        z: float,
        w: float,
        **kwargs
    ) -> typing.Tuple[float, float, float]:
    """
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
    """

    q = collections.namedtuple('Quaternion', ('x', 'y', 'z', 'w'))(
        x = x,
        y = y,
        z = z,
        w = w,
    )
    # maybe change input parsing later

    sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = np.arctan2(sinr_cosp, cosr_cosp);

    sinp = np.sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    cosp = np.sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    pitch = 2 * np.arctan2(sinp, cosp) - np.pi / 2;

    siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yaw = np.arctan2(siny_cosp, cosy_cosp);

    return roll, pitch, yaw
