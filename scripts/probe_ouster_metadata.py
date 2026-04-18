#!/usr/bin/env python3
"""Fetch Ouster sensor metadata over HTTP and pretty-print the fields the
Nebula decoder relies on (beam geometry, pixel_shift_by_row, data layout).
"""
import json
import sys
import urllib.request

IP = sys.argv[1] if len(sys.argv) > 1 else '169.254.12.72'

url = f'http://{IP}/api/v1/sensor/metadata'
print(f'Fetching {url}')
with urllib.request.urlopen(url, timeout=5) as r:
    d = json.load(r)

ldf = d.get('lidar_data_format') or d['sensor_info']['lidar_data_format']
bi = d.get('beam_intrinsics') or d['sensor_info']['beam_intrinsics']
cfg = d.get('config_params') or d['sensor_info']['config_params']

print(f"lidar_mode:         {cfg.get('lidar_mode')}")
print(f"udp_profile_lidar:  {cfg.get('udp_profile_lidar')}")
print(f"columns_per_frame:  {ldf['columns_per_frame']}")
print(f"columns_per_packet: {ldf['columns_per_packet']}")
print(f"pixels_per_column:  {ldf['pixels_per_column']}")

shift = ldf['pixel_shift_by_row']
print(f"\npixel_shift_by_row: len={len(shift)} min={min(shift)} max={max(shift)}")
print(f"  first 10: {shift[:10]}")
print(f"  last  10: {shift[-10:]}")

alt = bi['beam_altitude_angles']
az = bi['beam_azimuth_angles']
print(f"\nbeam_altitude_angles: len={len(alt)} min={min(alt):.2f} max={max(alt):.2f}")
print(f"  first 4: {alt[:4]}")
print(f"beam_azimuth_angles:  len={len(az)}  min={min(az):.2f} max={max(az):.2f}")
print(f"  first 8: {az[:8]}")
print(f"lidar_origin_to_beam_origin_mm: {bi.get('lidar_origin_to_beam_origin_mm')}")
