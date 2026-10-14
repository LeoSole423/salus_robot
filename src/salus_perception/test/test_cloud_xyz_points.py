"""Characterize the removed Python conversion, including binary layouts."""
import copy
import struct

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from salus_perception.scan_ground_filter import cloud_xyz_points
from salus_perception.scan_filters import rotate_translate_points
from salus_perception.radial_ground import RadialGroundConfig, non_ground_points


@pytest.mark.parametrize('dense', [True, False])
@pytest.mark.parametrize('bigendian', [True, False])
@pytest.mark.parametrize('double', [True, False])
@pytest.mark.parametrize('empty', [True, False])
def test_matches_original_conversion_and_radial_output(dense, bigendian, double, empty):
    datatype = PointField.FLOAT64 if double else PointField.FLOAT32
    size = 8 if double else 4
    fields = [PointField(name=name, offset=index*size, datatype=datatype, count=1)
              for index, name in enumerate(('intensity', 'z', 'x', 'y'))]
    values = [] if empty else [(1, 0, 1, 0), (2, .3, 2, 1),
                              (3, float('nan'), 2, 0), (4, float('inf'), 2, 0),
                              (5, -.2, 20, 0), (6, .1, .9, 0)]
    step=5*size  # trailing padding, extra field, reordered XYZ
    fmt=('>' if bigendian else '<')+('dddd' if double else 'ffff')
    data=b''.join(struct.pack(fmt,*v)+bytes(size) for v in values)
    msg=PointCloud2(header=Header(frame_id='lidar_link'),height=1,width=len(values),
                    fields=fields,is_bigendian=bigendian,point_step=step,
                    row_step=step*len(values),data=data,is_dense=dense)
    old=np.asarray([(float(r[0]),float(r[1]),float(r[2]))
                    for r in point_cloud2.read_points(copy.deepcopy(msg),
                    field_names=('x','y','z'),skip_nans=True)],dtype=np.float64).reshape(-1,3)
    new=cloud_xyz_points(copy.deepcopy(msg))
    assert new.dtype==np.float64
    np.testing.assert_array_equal(new,old)
    transform=dict(quaternion_xyzw=(0,0,0,1),translation_xyz=(0,0,.05))
    for config in [RadialGroundConfig(),RadialGroundConfig(global_slope_max_angle_deg=15,
                    local_slope_max_angle_deg=18,split_height_distance=.25)]:
        old_obs=non_ground_points(rotate_translate_points(old,**transform),config)
        new_obs=non_ground_points(rotate_translate_points(new,**transform),config)
        old_msg=point_cloud2.create_cloud_xyz32(Header(),old_obs.tolist())
        new_msg=point_cloud2.create_cloud_xyz32(Header(),new_obs.tolist())
        assert bytes(new_msg.data)==bytes(old_msg.data)
