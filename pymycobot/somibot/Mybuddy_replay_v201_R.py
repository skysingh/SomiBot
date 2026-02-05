#!/usr/bin/env python3
"""
MyBuddy 2 Cobot - Replay Only Script
Version 201_R - Teensy Serial Integration

This is a simplified replay-only version of the training script.
Use Mybuddy_train_v201_R.py to create recordings.

Usage:
  python3 Mybuddy_replay_v201_R.py recording.json
  python3 Mybuddy_replay_v201_R.py recording.csv --speed 30 --loop

Gripper Configuration:
- Left arm: Flexible Gripper
- Right arm: Parallel Gripper OR Vacuum Gripper (selectable via --gripper)

Teensy Serial (USB):
- Auto-detects on /dev/teensy (udev symlink)
- Handles pump channel actions recorded in sequence
"""

from pymycobot import MyBuddy
import time
import json
import csv
import os
import argparse
import serial
import threading
import RPi.GPIO as GPIO

# GPIO Pins - Outputs only (no inputs needed for replay)
VACUUM_PIN = 7
AUX1_PIN = 8
AUX2_PIN = 25
AUX3_PIN = 24
AUX4_PIN = 23
AUX5_PIN = 18
AUX_PINS = [AUX1_PIN, AUX2_PIN, AUX3_PIN, AUX4_PIN, AUX5_PIN]

# Teensy Serial Configuration
TEENSY_PORT = '/dev/teensy'
TEENSY_BAUD = 115200
TEENSY_TIMEOUT = 30

# LED Colors
LED_BLUE = [0, 0, 255]
LED_RED = [255, 0, 0]

# Gripper Constants
GRIPPER_SPEED = 50

# Gripper types
RIGHT_GRIPPER_PARALLEL = 'parallel'
RIGHT_GRIPPER_VACUUM = 'vacuum'


class MyBuddyReplay:
    def __init__(self, port='/dev/ttyACM0', baud=115200, right_gripper_type='vacuum'):
        print("Initializing MyBuddy 2 (Replay Mode)...")
        self.mybuddy = MyBuddy(port, baud)
        time.sleep(2)

        self.recorded_positions = []
        self.right_gripper_type = right_gripper_type
        self.aux_states = [0, 0, 0, 0, 0]

        # Teensy serial communication
        self.teensy = None
        self.teensy_status = {}
        self.teensy_lock = threading.Lock()
        teensy_ports = ['/dev/teensy', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyUSB0']
        for tport in teensy_ports:
            try:
                self.teensy = serial.Serial(tport, TEENSY_BAUD, timeout=0.1)
                self.teensy.reset_input_buffer()
                print(f"  ✓ Teensy connected on {tport}")
                break
            except:
                continue
        if not self.teensy:
            print(f"  ⚠ Teensy not found")

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Outputs only
        GPIO.setup(VACUUM_PIN, GPIO.OUT)
        GPIO.output(VACUUM_PIN, GPIO.LOW)
        for pin in AUX_PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        print("  ✓ GPIO outputs initialized")

        # Enable servos
        print("Enabling servos...")
        for arm_id in [1, 2]:
            for joint in range(1, 8):
                self.mybuddy.focus_servo(arm_id, joint)
                time.sleep(0.05)

        self.mybuddy.focus_servo(3, 1)
        time.sleep(0.1)
        self.mybuddy.power_on(3)
        time.sleep(0.5)

        self.set_led_color(*LED_BLUE)
        print("✓ Ready for replay\n")

    def cleanup(self):
        GPIO.output(VACUUM_PIN, GPIO.LOW)
        for pin in AUX_PINS:
            GPIO.output(pin, GPIO.LOW)
        if self.teensy:
            try:
                self.teensy.close()
            except:
                pass
        self.set_led_color(0, 0, 0)
        GPIO.cleanup()

    def set_led_color(self, r, g, b):
        try:
            self.mybuddy.set_tool_color(1, r, g, b)
            self.mybuddy.set_tool_color(2, r, g, b)
        except:
            pass

    def set_aux(self, aux_num, state):
        if aux_num < 1 or aux_num > 5:
            return
        pin = AUX_PINS[aux_num - 1]
        GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
        self.aux_states[aux_num - 1] = state

    # === TEENSY COMMUNICATION ===
    def teensy_send(self, cmd):
        if not self.teensy:
            return False
        try:
            self.teensy.reset_input_buffer()
            time.sleep(0.05)
            self.teensy.write(f"{cmd}\n".encode())
            self.teensy.flush()
            print(f"  → Teensy TX: {cmd}")
            return True
        except Exception as e:
            print(f"  ✗ Teensy send error: {e}")
            return False

    def teensy_read_status(self):
        if not self.teensy:
            return None
        try:
            if self.teensy.in_waiting > 0:
                line = self.teensy.readline().decode().strip()
                if line.startswith('{'):
                    with self.teensy_lock:
                        self.teensy_status = json.loads(line)
                    return self.teensy_status
        except:
            pass
        return None

    def teensy_start_channel(self, channel):
        if channel < 1 or channel > 4:
            return False
        return self.teensy_send(f"START{channel}")

    def teensy_wait_complete(self, channel, timeout=TEENSY_TIMEOUT):
        if not self.teensy:
            print("  ⚠ Teensy not connected - skipping wait")
            return "SKIPPED"

        print(f"  ⏳ Waiting for Teensy CH{channel} (timeout {timeout}s)...")
        start_time = time.time()
        last_state = None

        while time.time() - start_time < timeout:
            self.teensy_read_status()

            with self.teensy_lock:
                if self.teensy_status and 'channels' in self.teensy_status:
                    ch_data = self.teensy_status['channels'][channel - 1]
                    state = ch_data.get('state', 'IDLE')
                    ml = ch_data.get('ml', 0)

                    if state != last_state:
                        print(f"  → CH{channel}: {state} ({ml:.1f} mL)")
                        last_state = state

                    if state == 'COMPLETE':
                        print(f"  ✓ CH{channel} COMPLETE ({ml:.1f} mL)")
                        return "COMPLETE"
                    elif state == 'FAILED':
                        print(f"  ✗ CH{channel} FAILED ({ml:.1f} mL)")
                        return "FAILED"

            time.sleep(0.2)

        print(f"  ✗ CH{channel} TIMEOUT after {timeout}s")
        return "TIMEOUT"

    # === LOAD FUNCTIONS ===
    def load_from_json(self, filename):
        try:
            with open(filename, 'r') as f:
                data = json.load(f)

            self.recorded_positions = data['positions']
            total = len(self.recorded_positions)

            # Count types
            arm_count = sum(1 for p in self.recorded_positions if p.get('type') != 'teensy')
            teensy_count = sum(1 for p in self.recorded_positions if p.get('type') == 'teensy')

            print(f"✓ Loaded {total} positions from {filename}")
            print(f"  Arm positions: {arm_count}")
            if teensy_count > 0:
                print(f"  Teensy actions: {teensy_count}")

            return True
        except FileNotFoundError:
            print(f"✗ File not found: {filename}")
            return False
        except Exception as e:
            print(f"✗ Load error: {e}")
            return False

    def load_from_csv(self, filename):
        try:
            self.recorded_positions = []

            with open(filename, 'r') as f:
                reader = csv.DictReader(f)

                has_type_column = 'type' in reader.fieldnames
                has_delay_column = 'delay' in reader.fieldnames
                has_gripper_column = 'gripper' in reader.fieldnames
                has_aux_columns = all(f'aux{i}' in reader.fieldnames for i in range(1, 6))
                has_teensy_ch = 'teensy_ch' in reader.fieldnames

                for row in reader:
                    # Handle Teensy actions
                    if has_type_column and row.get('type', '').strip().lower() == 'teensy':
                        channel = int(row.get('teensy_ch', 1)) if has_teensy_ch else 1
                        delay = float(row['delay']) if has_delay_column and row.get('delay') else 0.5
                        self.recorded_positions.append({
                            'type': 'teensy',
                            'channel': channel,
                            'delay': delay
                        })
                        continue

                    # Regular arm position
                    angles = [float(row[f'j{j}']) for j in range(1, 7)]
                    arm = row['arm'].strip().lower()
                    waist = float(row['waist'])
                    gripper = int(float(row['gripper'])) if has_gripper_column else 0
                    gripper_type = row.get('gripper_type', 'parallel').strip().lower()
                    delay = float(row['delay']) if has_delay_column else 5.0
                    aux = [int(row[f'aux{j}']) for j in range(1, 6)] if has_aux_columns else [0]*5

                    self.recorded_positions.append({
                        'arm': arm,
                        'angles': angles,
                        'waist': waist,
                        'gripper': gripper,
                        'gripper_type': gripper_type,
                        'aux': aux,
                        'delay': delay
                    })

            total = len(self.recorded_positions)
            arm_count = sum(1 for p in self.recorded_positions if p.get('type') != 'teensy')
            teensy_count = sum(1 for p in self.recorded_positions if p.get('type') == 'teensy')

            print(f"✓ Loaded {total} positions from {filename}")
            print(f"  Arm positions: {arm_count}")
            if teensy_count > 0:
                print(f"  Teensy actions: {teensy_count}")

            return True
        except FileNotFoundError:
            print(f"✗ File not found: {filename}")
            return False
        except Exception as e:
            print(f"✗ Load error: {e}")
            return False

    # === REPLAY ===
    def replay(self, speed=50, loop=False):
        if not self.recorded_positions:
            print("No recording loaded")
            return

        print(f"\n▶️  REPLAY - {len(self.recorded_positions)} positions")
        print(f"   Speed: {speed} | Loop: {loop}\n")

        self.mybuddy.focus_servo(3, 1)
        time.sleep(0.1)
        self.mybuddy.power_on(3)
        time.sleep(0.3)

        self.set_led_color(*LED_RED)

        try:
            iteration = 1
            while True:
                if loop:
                    print(f"\n{'='*60}")
                    print(f"Iteration {iteration}")
                    print(f"{'='*60}\n")

                replay_start = time.time()

                for i, pos in enumerate(self.recorded_positions):
                    elapsed = time.time() - replay_start

                    # Handle Teensy actions
                    if pos.get('type') == 'teensy':
                        channel = pos.get('channel', 1)
                        print(f"[{elapsed:6.1f}s] Position {i+1}/{len(self.recorded_positions)} | TEENSY CH{channel}")

                        if self.teensy_start_channel(channel):
                            result = self.teensy_wait_complete(channel)
                            if result == "COMPLETE":
                                print(f"  ✓ CH{channel} completed")
                            elif result == "FAILED":
                                print(f"  ✗ CH{channel} failed")
                            elif result == "TIMEOUT":
                                print(f"  ⚠ CH{channel} timed out")
                        continue

                    # Regular arm position
                    arm_id = 1 if pos['arm'] == 'left' else 2
                    arm_name = pos['arm'].upper()
                    waist = pos.get('waist', 0.0)
                    gripper = pos.get('gripper', 0)
                    gripper_type = pos.get('gripper_type', 'parallel')
                    delay = pos.get('delay', 0.5)
                    aux = pos.get('aux', [0, 0, 0, 0, 0])

                    angles_str = ", ".join([f"{a:6.1f}°" for a in pos['angles']])
                    aux_str = "".join([str(a) for a in aux])

                    print(f"[{elapsed:6.1f}s] Position {i+1}/{len(self.recorded_positions)} | {arm_name:5s} | "
                          f"Waist: {waist:6.1f}° | AUX: [{aux_str}]")

                    # Set AUX outputs
                    for j, state in enumerate(aux):
                        if self.aux_states[j] != state:
                            self.set_aux(j + 1, state)

                    # Move waist
                    try:
                        self.mybuddy.send_angle(3, 1, waist, speed)
                        time.sleep(0.1)
                    except Exception as e:
                        print(f"  ⚠ Waist error: {e}")

                    # Move arm
                    self.mybuddy.send_angles(arm_id, pos['angles'], speed)
                    time.sleep(delay)

                    # Set gripper
                    if gripper_type == 'vacuum':
                        if gripper >= 50:
                            GPIO.output(VACUUM_PIN, GPIO.HIGH)
                        else:
                            GPIO.output(VACUUM_PIN, GPIO.LOW)
                        time.sleep(0.3)
                    else:
                        try:
                            self.mybuddy.set_gripper_value(arm_id, gripper, GRIPPER_SPEED)
                            time.sleep(1.5)
                        except:
                            pass

                total_time = time.time() - replay_start
                print(f"\n{'='*60}")
                print(f"✓ Replay complete - {total_time:.1f} seconds")
                print(f"{'='*60}")

                if not loop:
                    break

                iteration += 1
                time.sleep(1)

        except KeyboardInterrupt:
            print("\n⏹ Stopped")
        finally:
            GPIO.output(VACUUM_PIN, GPIO.LOW)
            for pin in AUX_PINS:
                GPIO.output(pin, GPIO.LOW)
            self.aux_states = [0, 0, 0, 0, 0]
            self.set_led_color(*LED_BLUE)


def main():
    parser = argparse.ArgumentParser(description='MyBuddy 2 Replay Script v201_R')
    parser.add_argument('recording', help='Recording file (JSON or CSV)')
    parser.add_argument('--gripper', choices=['parallel', 'vacuum'], default='vacuum',
                        help='Right arm gripper type (default: vacuum)')
    parser.add_argument('--speed', type=int, default=50, help='Replay speed (default: 50)')
    parser.add_argument('--loop', action='store_true', help='Loop replay continuously')
    parser.add_argument('--port', default='/dev/ttyACM0', help='MyBuddy serial port')

    args = parser.parse_args()

    print("="*60)
    print("  MyBuddy 2 Replay System v201_R")
    print("="*60)

    if not os.path.exists(args.recording):
        print(f"✗ File not found: {args.recording}")
        return

    player = MyBuddyReplay(port=args.port, right_gripper_type=args.gripper)

    try:
        # Load recording
        if args.recording.endswith('.json'):
            if not player.load_from_json(args.recording):
                return
        elif args.recording.endswith('.csv'):
            if not player.load_from_csv(args.recording):
                return
        else:
            print("✗ Unknown file format (use .json or .csv)")
            return

        # Run replay
        player.replay(speed=args.speed, loop=args.loop)

    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        player.cleanup()
        print("Goodbye!")


if __name__ == "__main__":
    main()
