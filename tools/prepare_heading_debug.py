#!/usr/bin/env python3
"""Copy unchanged EKF numerical configs and enable native diagnostic logging."""
from pathlib import Path
import sys


def prepare(directory):
    directory = Path(directory)
    for name in ('local', 'global'):
        source = Path(f'src/salus_localization/config/localization_{name}_sim.yaml')
        config = source.read_text().replace('  ros__parameters:\n',
            f'  ros__parameters:\n    debug: true\n    debug_out_file: {directory.resolve()}/{name}-ekf-debug.log\n', 1)
        (directory/f'{name}-debug.yaml').write_text(config)

    global_source = Path('src/salus_localization/config/localization_global_sim.yaml').read_text()
    for name in ('no_gps_position', 'compensated_gps', 'delayed_gps_reference'):
        node_name = f'ekf_global_{name}'
        config = global_source.replace('ekf_filter_node_global:', node_name+':', 1)
        config = config.replace('    publish_tf: true', '    publish_tf: false')
        if name == 'no_gps_position':
            config = config.replace('odom1_config: [true, true,', 'odom1_config: [false, false,')
        else:
            topic = '/diagnostic/gps_base_position' if name == 'compensated_gps' else '/diagnostic/gps_ant_position_delayed'
            config = config.replace('odom1: /gps/odometry_map', f'odom1: {topic}')
        config = config.replace('  ros__parameters:\n',
            f'  ros__parameters:\n    debug: true\n    debug_out_file: {directory.resolve()}/{name}-ekf-debug.log\n', 1)
        (directory/f'{name}-debug.yaml').write_text(config)


if __name__ == '__main__':
    prepare(sys.argv[1])
