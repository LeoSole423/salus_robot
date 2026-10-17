"""Pure gyro rotation and bounded-window comparison statistics."""

import math


def rotate(values, quaternion):
    if not all(math.isfinite(v) for v in (*values, *quaternion)):
        raise ValueError('nonfinite vector/rotation')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-9:
        raise ValueError('degenerate rotation')
    x, y, z, w = (v / norm for v in quaternion)
    a, b, c = values
    return ((1-2*(y*y+z*z))*a + 2*(x*y-z*w)*b + 2*(x*z+y*w)*c,
            2*(x*y+z*w)*a + (1-2*(x*x+z*z))*b + 2*(y*z-x*w)*c,
            2*(x*z-y*w)*a + 2*(y*z+x*w)*b + (1-2*(x*x+y*y))*c)


def metrics(errors):
    if not errors:
        return {'pairs': 0, 'mean_error_rad_s': None, 'rms_error_rad_s': None}
    return {'pairs': len(errors),
            'mean_error_rad_s': [sum(e[i] for e in errors)/len(errors) for i in range(3)],
            'rms_error_rad_s': [math.sqrt(sum(e[i]**2 for e in errors)/len(errors))
                                for i in range(3)]}
