#!/usr/bin/env python3
"""Subscribe to /points and print an azimuth histogram to diagnose 4-ghost issues.

A correct 360-degree scan has points roughly evenly distributed across all 8 bins.
A 4-ghost bug would show points concentrated in only 2 bins (180-degree coverage)
or 4 bins (every other bin), with others empty.
"""
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

rclpy.init()
node = rclpy.create_node('probe_hist')
msg_holder = [None]
node.create_subscription(
    PointCloud2, '/points',
    lambda m: msg_holder.__setitem__(0, m), qos_profile_sensor_data)

for _ in range(40):
    rclpy.spin_once(node, timeout_sec=0.5)
    if msg_holder[0] is not None:
        break

if msg_holder[0] is None:
    print('No /points message received. Is the driver running?')
    raise SystemExit(1)

msg = msg_holder[0]
pts = np.array(list(pc2.read_points(
    msg, field_names=['x', 'y', 'azimuth'], skip_nans=True)))

print(f'Total points: {len(pts)}')
print(f'X range: [{pts["x"].min():.2f}, {pts["x"].max():.2f}]')
print(f'Y range: [{pts["y"].min():.2f}, {pts["y"].max():.2f}]')

# Azimuth from stored field (radians)
az_rad = pts['azimuth']
az_deg = (np.degrees(az_rad) + 360) % 360

# Also compute azimuth from xy (should match)
az_deg_xy = (np.degrees(np.arctan2(pts['y'], pts['x'])) + 360) % 360

print(f'\nAzimuth histogram (from stored azimuth field):')
hist, _ = np.histogram(az_deg, bins=8, range=(0, 360))
for i, c in enumerate(hist):
    bar = '#' * int(c / max(hist.max(), 1) * 40)
    print(f'  {i*45:3d}-{(i+1)*45:3d} deg: {c:6d}  {bar}')

print(f'\nAzimuth histogram (recomputed from XY):')
hist_xy, _ = np.histogram(az_deg_xy, bins=8, range=(0, 360))
for i, c in enumerate(hist_xy):
    bar = '#' * int(c / max(hist_xy.max(), 1) * 40)
    print(f'  {i*45:3d}-{(i+1)*45:3d} deg: {c:6d}  {bar}')

rclpy.shutdown()
