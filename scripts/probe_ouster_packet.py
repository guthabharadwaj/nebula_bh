#!/usr/bin/env python3
"""Bind to the Ouster UDP port and decode the first packet's column headers.

Run this only when the Nebula node is STOPPED (otherwise the port is busy).
"""
import socket
import struct
import sys

HOST = sys.argv[1] if len(sys.argv) > 1 else '169.254.108.194'
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 7502

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((HOST, PORT))
s.settimeout(5)
data, src = s.recvfrom(65535)
print(f'Received {len(data)} bytes from {src}')

# Packet header (modern profile): 32 bytes
print(f"Packet header: packet_type={struct.unpack_from('<H', data, 0)[0]} "
      f"frame_id={struct.unpack_from('<H', data, 2)[0]}")

# 16 column headers (each 12 bytes), each followed by 128 pixels x 12 bytes
for i in range(16):
    off = 32 + i * (12 + 12 * 128)
    ts = struct.unpack_from('<Q', data, off)[0]
    mid = struct.unpack_from('<H', data, off + 8)[0]
    status = struct.unpack_from('<H', data, off + 10)[0]
    print(f'col {i}: measurement_id={mid}, status=0x{status:04x}, ts={ts}')
