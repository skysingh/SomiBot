#!/usr/bin/env python3
"""Quick test for RPi-Teensy USB serial communication"""

import serial
import time
import json

# Try to find Teensy
ports = ['/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyUSB0', '/dev/ttyACM0']

teensy = None
for port in ports:
    try:
        teensy = serial.Serial(port, 115200, timeout=1)
        print(f"✓ Connected to {port}")
        break
    except:
        print(f"  - {port} not available")

if not teensy:
    print("✗ Teensy not found!")
    exit(1)

time.sleep(0.5)
teensy.reset_input_buffer()

print("\n--- Reading status for 2 seconds ---\n")

start = time.time()
while time.time() - start < 2:
    try:
        line = teensy.readline().decode().strip()
        if line.startswith('{'):
            data = json.loads(line)
            ch1 = data['channels'][0]
            print(f"CH1: {ch1['state']:8s} latched={ch1['latched']} ml={ch1['ml']:.1f}")
    except:
        pass

print("\n--- Sending START1 ---")
teensy.reset_input_buffer()
time.sleep(0.1)
teensy.write(b"START1\n")
teensy.flush()

print("--- Waiting 3 seconds for response ---\n")
start = time.time()
while time.time() - start < 3:
    try:
        line = teensy.readline().decode().strip()
        if 'REMOTE' in line or 'RX:' in line or 'Latched' in line:
            print(f">>> {line}")
        elif line.startswith('{'):
            data = json.loads(line)
            ch1 = data['channels'][0]
            print(f"CH1: {ch1['state']:8s} latched={ch1['latched']} ml={ch1['ml']:.1f}")
    except:
        pass

print("\n--- Sending STOP1 ---")
teensy.write(b"STOP1\n")
time.sleep(0.5)

teensy.close()
print("\n✓ Done")
