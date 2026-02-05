#!/usr/bin/env python3
"""
MyBuddy 2 Cobot - Training and Replay Script
Version 200_R - Swapped Gripper Configuration

Gripper Configuration:
- Left arm: Flexible Gripper
- Right arm: Parallel Gripper OR Vacuum Gripper (selectable)

CONFIRMED ARM IDs:
- Left arm: ID 1
- Right arm: ID 2  
- Waist: ID 3

GPIO Inputs:
- Pin 17: Deadman switch (hold to move arm)
- Pin 27: Record button (press when arm locked)
- Pin 22: Right gripper toggle (parallel) / Vacuum toggle

GPIO Outputs:
- Pin 7:  Vacuum pump control (HIGH=ON, LOW=OFF)
- Pin 8:  AUX1 (a1on/a1off)
- Pin 25: AUX2 (a2on/a2off)
- Pin 24: AUX3 (a3on/a3off)
- Pin 23: AUX4 (a4on/a4off)
- Pin 18: AUX5 (a5on/a5off)

CSV Format:
arm,waist,gripper,gripper_type,j1,j2,j3,j4,j5,j6,delay,aux1,aux2,aux3,aux4,aux5

v134 Changes:
- SWAPPED GRIPPERS:
  - Left arm: Flexible Gripper (was right)
  - Right arm: Parallel OR Vacuum (was left)

LED Status Indicators:
- BLUE: Ready/idle
- GREEN: Deadman active (arms free to move)
- YELLOW: Recording position (flash)
- RED: Replaying sequence
"""

from pymycobot import MyBuddy
import time
import json
import csv
import os
import argparse
import serial
import threading
from datetime import datetime
import RPi.GPIO as GPIO
import numpy as np

# GPIO Pins - Inputs
DEADMAN_PIN = 17
RECORD_PIN = 27
LEFT_GRIPPER_PIN = 22

# GPIO Pins - Outputs
VACUUM_PIN = 7
AUX1_PIN = 8
AUX2_PIN = 25
AUX3_PIN = 24
AUX4_PIN = 23
AUX5_PIN = 18

# Auxiliary pins list for easy iteration
AUX_PINS = [AUX1_PIN, AUX2_PIN, AUX3_PIN, AUX4_PIN, AUX5_PIN]

# Teensy Serial Configuration
TEENSY_PORT = '/dev/serial0'  # TX1/RX1 on RPi
TEENSY_BAUD = 115200
TEENSY_TIMEOUT = 30  # Seconds to wait for COMPLETE/FAILED

# LED Colors
LED_BLUE = [0, 0, 255]    # Ready
LED_GREEN = [0, 255, 0]   # Moving (deadman active)
LED_YELLOW = [255, 255, 0] # Recording
LED_RED = [255, 0, 0]     # Replaying

# Gripper Constants
GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0
GRIPPER_SPEED = 50

# Vacuum Constants
VACUUM_ON = 100
VACUUM_OFF = 0

# Right arm gripper types (selectable)
RIGHT_GRIPPER_PARALLEL = 'parallel'
RIGHT_GRIPPER_VACUUM = 'vacuum'

class MyBuddyTrainer:
    def __init__(self, port='/dev/ttyACM0', baud=115200, right_gripper_type='parallel'):
        print("Initializing MyBuddy 2...")
        self.mybuddy = MyBuddy(port, baud)
        time.sleep(2)
        
        self.recorded_positions = []
        self.deadman_active = False
        self.last_record_time = 0
        self.record_debounce = 0.3
        self.last_left_gripper_time = 0
        self.gripper_debounce = 0.3
        
        # Left arm gripper type
        self.right_gripper_type = right_gripper_type
        
        # Track gripper states
        self.left_gripper_state = None
        self.right_gripper_state = None
        self.vacuum_state = False
        
        # Auxiliary output states (all OFF at startup)
        self.aux_states = [0, 0, 0, 0, 0]  # aux1-aux5

        # Teensy serial communication
        self.teensy = None
        self.teensy_status = {}
        self.teensy_lock = threading.Lock()
        try:
            self.teensy = serial.Serial(TEENSY_PORT, TEENSY_BAUD, timeout=0.1)
            self.teensy.reset_input_buffer()
            print(f"  ✓ Teensy connected on {TEENSY_PORT}")
        except Exception as e:
            print(f"  ⚠ Teensy not available: {e}")

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Inputs
        GPIO.setup(DEADMAN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RECORD_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LEFT_GRIPPER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Outputs - Vacuum
        GPIO.setup(VACUUM_PIN, GPIO.OUT)
        GPIO.output(VACUUM_PIN, GPIO.LOW)
        
        # Outputs - Auxiliary (all OFF at startup)
        for pin in AUX_PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        print("  ✓ All auxiliary outputs OFF")
        
        # Enable all servos at startup
        print("Enabling servos...")
        for arm_id in [1, 2]:
            for joint in range(1, 8):
                self.mybuddy.focus_servo(arm_id, joint)
                time.sleep(0.05)
        
        # Enable waist servo AND power it on
        print("Enabling waist...")
        self.mybuddy.focus_servo(3, 1)
        time.sleep(0.1)
        self.mybuddy.power_on(3)
        time.sleep(0.5)
        
        # Give servos time to stabilize
        print("Waiting for servos to stabilize...")
        time.sleep(1.0)
        
        # Test read positions
        print("Testing position reading...")
        test_left = self.mybuddy.get_angles(1)
        test_right = self.mybuddy.get_angles(2)
        test_waist = self.mybuddy.get_angles(3)
        print(f"  Left arm: {test_left}")
        print(f"  Right arm: {test_right}")
        print(f"  Waist: {test_waist}")
        
        if not test_left or len(test_left) == 0:
            print("  ⚠ Warning: Left arm not responding")
        if not test_right or len(test_right) == 0:
            print("  ⚠ Warning: Right arm not responding")
        if not test_waist or len(test_waist) == 0:
            print("  ⚠ Warning: Waist not responding")

        # Store home position (startup position)
        self.home_left = test_left if test_left else []
        self.home_right = test_right if test_right else []
        self.home_waist = test_waist[0] if test_waist else 0
        print(f"  ✓ Home position saved")
        
        # Test gripper communication
        print("Testing grippers...")
        
        # Left arm - Flexible gripper
        left_val = self.mybuddy.get_gripper_value(1)
        print(f"  Left gripper (flexible): {left_val}")
        if left_val == -1:
            print("  ⚠ Warning: Left gripper not responding")
        else:
            print("  ✓ Left gripper ready")
        
        # Right arm - Parallel or Vacuum
        print(f"  Right arm gripper type: {self.right_gripper_type.upper()}")
        if self.right_gripper_type == RIGHT_GRIPPER_PARALLEL:
            right_val = self.mybuddy.get_gripper_value(2)
            print(f"  Right gripper (parallel): {right_val}")
            if right_val == -1:
                print("  ⚠ Warning: Right gripper not responding")
            else:
                print("  ✓ Right gripper ready")
        else:
            print(f"  Right gripper (vacuum): GPIO {VACUUM_PIN}")
            print("  ✓ Vacuum pump ready")
        
        # Show aux pin assignments
        print("Auxiliary outputs:")
        print(f"  AUX1: GPIO {AUX1_PIN}")
        print(f"  AUX2: GPIO {AUX2_PIN}")
        print(f"  AUX3: GPIO {AUX3_PIN}")
        print(f"  AUX4: GPIO {AUX4_PIN}")
        print(f"  AUX5: GPIO {AUX5_PIN}")
        
        self.set_led_color(*LED_BLUE)
        print("✓ System ready!\n")
    
    def cleanup(self):
        # Turn off vacuum pump
        GPIO.output(VACUUM_PIN, GPIO.LOW)
        # Turn off all auxiliary outputs
        for pin in AUX_PINS:
            GPIO.output(pin, GPIO.LOW)
        # Close Teensy serial connection
        if self.teensy:
            try:
                self.teensy.close()
                print("  ✓ Teensy disconnected")
            except:
                pass
        self.set_led_color(0, 0, 0)
        GPIO.cleanup()
    
    # === LED CONTROL ===
    def set_led_color(self, r, g, b, arm=None):
        if arm is None:
            try:
                self.mybuddy.set_tool_color(1, r, g, b)
                self.mybuddy.set_tool_color(2, r, g, b)
            except:
                pass
        else:
            try:
                self.mybuddy.set_tool_color(arm, r, g, b)
            except:
                pass
    
    # === AUXILIARY OUTPUT CONTROL ===
    def set_aux(self, aux_num, state):
        """Set auxiliary output (1-5) to state (0=OFF, 1=ON)"""
        if aux_num < 1 or aux_num > 5:
            print(f"  ⚠ Invalid aux number: {aux_num} (use 1-5)")
            return
        
        idx = aux_num - 1
        pin = AUX_PINS[idx]
        
        if state:
            GPIO.output(pin, GPIO.HIGH)
            self.aux_states[idx] = 1
            print(f"  ✓ AUX{aux_num} (GPIO {pin}): ON")
        else:
            GPIO.output(pin, GPIO.LOW)
            self.aux_states[idx] = 0
            print(f"  ✓ AUX{aux_num} (GPIO {pin}): OFF")
    
    def toggle_aux(self, aux_num):
        """Toggle auxiliary output (1-5)"""
        if aux_num < 1 or aux_num > 5:
            print(f"  ⚠ Invalid aux number: {aux_num} (use 1-5)")
            return
        
        idx = aux_num - 1
        new_state = 0 if self.aux_states[idx] else 1
        self.set_aux(aux_num, new_state)
    
    def get_aux_states(self):
        """Get current aux states as list [aux1, aux2, aux3, aux4, aux5]"""
        return self.aux_states.copy()
    
    def set_aux_states(self, states):
        """Set all aux states from list [aux1, aux2, aux3, aux4, aux5]"""
        for i, state in enumerate(states):
            if i < 5:
                self.set_aux(i + 1, state)
    
    def show_aux_status(self):
        """Display current aux output status"""
        print("\n  Auxiliary Output Status:")
        for i in range(5):
            state = "ON" if self.aux_states[i] else "OFF"
            print(f"    AUX{i+1} (GPIO {AUX_PINS[i]}): {state}")
        print()
    
    # === VACUUM CONTROL ===
    def vacuum_on(self):
        GPIO.output(VACUUM_PIN, GPIO.HIGH)
        self.vacuum_state = True
        self.right_gripper_state = VACUUM_ON
        print("  ✓ VACUUM: ON (GPIO 7 HIGH)")
    
    def vacuum_off(self):
        GPIO.output(VACUUM_PIN, GPIO.LOW)
        self.vacuum_state = False
        self.right_gripper_state = VACUUM_OFF
        print("  ✓ VACUUM: OFF (GPIO 7 LOW)")
    
    def toggle_vacuum(self):
        if self.vacuum_state:
            self.vacuum_off()
        else:
            self.vacuum_on()
    
    # === GRIPPER CONTROL ===
    def set_gripper(self, arm, value, speed=GRIPPER_SPEED):
        """
        Left arm: Flexible gripper (arm_id=1)
        Right arm: Parallel OR Vacuum gripper (arm_id=2)
        """
        if arm == 'both':
            print(f"Setting BOTH grippers to {value}...")
            # Left arm - always flexible
            try:
                self.mybuddy.set_gripper_value(1, value, speed)
                self.left_gripper_state = value
                time.sleep(0.2)
            except Exception as e:
                print(f"  Left gripper error: {e}")
            
            # Right arm - parallel or vacuum
            if self.right_gripper_type == RIGHT_GRIPPER_PARALLEL:
                try:
                    self.mybuddy.set_gripper_value(2, value, speed)
                    self.right_gripper_state = value
                    time.sleep(2.0)
                except Exception as e:
                    print(f"  Right gripper error: {e}")
            else:
                if value >= 50:
                    self.vacuum_on()
                else:
                    self.vacuum_off()
        
        elif arm == 'left':
            # Left arm - always flexible gripper
            print(f"Setting left gripper (flexible) to {value}...")
            try:
                self.mybuddy.set_gripper_value(1, value, speed)
                self.left_gripper_state = value
                time.sleep(2.0)
                actual = self.mybuddy.get_gripper_value(1)
                print(f"  → Actual position: {actual}")
            except Exception as e:
                print(f"  Error: {e}")
        
        else:  # right
            # Right arm - parallel or vacuum
            if self.right_gripper_type == RIGHT_GRIPPER_PARALLEL:
                print(f"Setting right gripper (parallel) to {value}...")
                try:
                    self.mybuddy.set_gripper_value(2, value, speed)
                    self.right_gripper_state = value
                    time.sleep(2.0)
                    actual = self.mybuddy.get_gripper_value(2)
                    print(f"  → Actual position: {actual}")
                except Exception as e:
                    print(f"  Error: {e}")
            else:
                # Vacuum gripper
                if value >= 50:
                    self.vacuum_on()
                else:
                    self.vacuum_off()
    
    def open_gripper(self, arm):
        self.set_gripper(arm, GRIPPER_OPEN)
        if arm == 'right' and self.right_gripper_type == RIGHT_GRIPPER_VACUUM:
            print(f"  ✓ RIGHT vacuum: ON")
        else:
            state = "OPEN" if arm != 'both' else "OPEN (both)"
            print(f"  ✓ {arm.upper()} gripper: {state}")
    
    def close_gripper(self, arm):
        self.set_gripper(arm, GRIPPER_CLOSED)
        if arm == 'right' and self.right_gripper_type == RIGHT_GRIPPER_VACUUM:
            print(f"  ✓ RIGHT vacuum: OFF")
        else:
            state = "CLOSED" if arm != 'both' else "CLOSED (both)"
            print(f"  ✓ {arm.upper()} gripper: {state}")
    
    def toggle_gripper(self, arm):
        if arm == 'left':
            # Left arm always has flexible gripper
            if self.left_gripper_state == GRIPPER_OPEN:
                self.close_gripper(arm)
            else:
                self.open_gripper(arm)
        elif arm == 'right':
            # Right arm has parallel or vacuum
            if self.right_gripper_type == RIGHT_GRIPPER_VACUUM:
                self.toggle_vacuum()
            else:
                if self.right_gripper_state == GRIPPER_OPEN:
                    self.close_gripper(arm)
                else:
                    self.open_gripper(arm)
        elif arm == 'both':
            if self.left_gripper_state != GRIPPER_OPEN or self.right_gripper_state != GRIPPER_OPEN:
                self.open_gripper(arm)
            else:
                self.close_gripper(arm)
    
    def get_gripper_state(self, arm):
        if arm == 'left':
            # Left arm always flexible
            try:
                value = self.mybuddy.get_gripper_value(1)
                return value if value is not None and value != -1 else 0
            except:
                return 0
        else:
            # Right arm - parallel or vacuum
            if self.right_gripper_type == RIGHT_GRIPPER_VACUUM:
                return VACUUM_ON if self.vacuum_state else VACUUM_OFF
            else:
                try:
                    value = self.mybuddy.get_gripper_value(2)
                    return value if value is not None and value != -1 else 0
                except:
                    return 0
    
    # === ARM CONTROL ===
    def release_arm(self, arm):
        if arm == 'both':
            print("Releasing BOTH arms...")
            self.mybuddy.release_all_servos(1)
            time.sleep(0.2)
            self.mybuddy.release_all_servos(2)
            time.sleep(0.3)
        else:
            arm_id = 1 if arm == 'left' else 2
            print(f"Releasing {arm} arm (ID {arm_id})...")
            self.mybuddy.release_all_servos(arm_id)
            time.sleep(0.5)
    
    def lock_arm(self, arm):
        if arm == 'both':
            print("Locking BOTH arms...")
            self.mybuddy.power_on(1)
            time.sleep(0.2)
            self.mybuddy.power_on(2)
            time.sleep(0.3)
        else:
            arm_id = 1 if arm == 'left' else 2
            print(f"Locking {arm} arm (ID {arm_id})...")
            self.mybuddy.power_on(arm_id)
            time.sleep(0.5)
    
    def read_position(self, arm):
        arm_id = 1 if arm == 'left' else 2
        
        for attempt in range(3):
            angles = self.mybuddy.get_angles(arm_id)
            
            if not angles or len(angles) == 0:
                print(f"⚠ Warning: Could not read {arm} arm position (attempt {attempt+1}/3)")
                time.sleep(0.2)
                continue
            
            if all(abs(a) < 0.1 for a in angles):
                print(f"⚠ Warning: {arm} arm returned all zeros (attempt {attempt+1}/3)")
                time.sleep(0.2)
                continue
            
            return angles
        
        print(f"❌ Error: Could not read valid {arm} arm position after 3 attempts")
        return [0, 0, 0, 0, 0, 0]
    
    def read_waist(self):
        for attempt in range(3):
            try:
                angles = self.mybuddy.get_angles(3)
                if angles and len(angles) > 0:
                    return angles[0]
                else:
                    print(f"⚠ Warning: Could not read waist position (attempt {attempt+1}/3)")
                    time.sleep(0.2)
            except Exception as e:
                print(f"⚠ Warning: Waist read error (attempt {attempt+1}/3): {e}")
                time.sleep(0.2)
        
        print(f"❌ Error: Could not read waist position after 3 attempts")
        return 0.0
    
    def set_waist(self, angle, speed=50):
        try:
            self.mybuddy.send_angle(3, 1, angle, speed)
            time.sleep(0.2)
            print(f"  ✓ Waist set to {angle:6.1f}°")
        except Exception as e:
            print(f"  ✗ Waist error: {e}")
    
    def release_waist(self):
        try:
            self.mybuddy.release_all_servos(3)
            time.sleep(0.3)
        except Exception as e:
            print(f"Waist release error: {e}")
    
    def lock_waist(self):
        try:
            self.mybuddy.power_on(3)
            time.sleep(0.3)
        except Exception as e:
            print(f"Waist lock error: {e}")

    # === HOME POSITION ===
    def go_home(self, speed=30):
        """Move all arms to home position"""
        print("\nMoving to home position...")
        try:
            if self.home_left and len(self.home_left) == 6:
                print("  → Left arm...")
                self.mybuddy.send_angles(1, self.home_left, speed)
            if self.home_right and len(self.home_right) == 6:
                print("  → Right arm...")
                self.mybuddy.send_angles(2, self.home_right, speed)
            if self.home_waist is not None:
                print("  → Waist...")
                self.mybuddy.send_angle(3, 1, self.home_waist, speed)
            time.sleep(2)
            print("  ✓ Home position reached\n")
        except Exception as e:
            print(f"  ✗ Error going home: {e}\n")

    def set_home(self):
        """Set current position as new home"""
        print("\nSetting current position as home...")
        try:
            self.home_left = self.mybuddy.get_angles(1)
            self.home_right = self.mybuddy.get_angles(2)
            waist = self.mybuddy.get_angles(3)
            self.home_waist = waist[0] if waist else 0
            print(f"  Left:  {self.home_left}")
            print(f"  Right: {self.home_right}")
            print(f"  Waist: {self.home_waist:.1f}°")
            print("  ✓ New home position saved\n")
        except Exception as e:
            print(f"  ✗ Error setting home: {e}\n")

    def show_home(self):
        """Display current home position"""
        print("\nHome position:")
        print(f"  Left:  {self.home_left}")
        print(f"  Right: {self.home_right}")
        print(f"  Waist: {self.home_waist:.1f}°\n")

    # === TEENSY COMMUNICATION ===
    def teensy_send(self, cmd):
        """Send command to Teensy"""
        if not self.teensy:
            print("  ⚠ Teensy not connected")
            return False
        try:
            self.teensy.write(f"{cmd}\n".encode())
            print(f"  → Teensy TX: {cmd}")
            return True
        except Exception as e:
            print(f"  ✗ Teensy send error: {e}")
            return False

    def teensy_read_status(self):
        """Read and parse JSON status from Teensy"""
        if not self.teensy:
            return None
        try:
            if self.teensy.in_waiting > 0:
                line = self.teensy.readline().decode().strip()
                if line.startswith('{'):
                    with self.teensy_lock:
                        self.teensy_status = json.loads(line)
                    return self.teensy_status
        except Exception as e:
            pass
        return None

    def teensy_start_channel(self, channel):
        """Start a Teensy channel (1-4) and record to sequence"""
        if channel < 1 or channel > 4:
            print(f"  ⚠ Invalid channel {channel} (use 1-4)")
            return False
        return self.teensy_send(f"START{channel}")

    def teensy_stop_channel(self, channel):
        """Stop a Teensy channel (1-4)"""
        if channel < 1 or channel > 4:
            print(f"  ⚠ Invalid channel {channel} (use 1-4)")
            return False
        return self.teensy_send(f"STOP{channel}")

    def teensy_set_setpoint(self, ml):
        """Set Teensy setpoint in mL"""
        if ml < 50 or ml > 500:
            print(f"  ⚠ Invalid setpoint {ml} (use 50-500)")
            return False
        return self.teensy_send(f"SP={ml}")

    def teensy_wait_complete(self, channel, timeout=TEENSY_TIMEOUT):
        """Wait for channel to complete or fail"""
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

    def record_teensy_action(self, channel):
        """Record a Teensy channel action to the sequence"""
        timestamp = time.time()
        time_from_start = timestamp - self.start_time

        self.recorded_positions.append({
            'type': 'teensy',
            'channel': channel,
            'timestamp': timestamp,
            'time_from_start': time_from_start,
            'delay': 0.5
        })
        print(f"✓ Recorded Teensy CH{channel} action (position {len(self.recorded_positions)})\n")

    # === RECORDING ===
    def record_position(self, arm):
        print(f"  → Reading waist position...")
        waist = self.read_waist()
        print(f"  → Waist: {waist:.2f}°")
        
        # Get current aux states
        aux = self.get_aux_states()
        aux_str = "".join([str(a) for a in aux])
        print(f"  → AUX: [{aux_str}]")
        
        timestamp = time.time()
        time_from_start = timestamp - self.start_time
        
        if arm == 'left':
            led_arm = 1
        elif arm == 'right':
            led_arm = 2
        else:
            led_arm = None
        
        if arm == 'both':
            print(f"  → Reading left arm position...")
            left_angles = self.read_position('left')
            print(f"  → Reading right arm position...")
            right_angles = self.read_position('right')
            left_gripper = self.left_gripper_state if self.left_gripper_state is not None else self.get_gripper_state('left')
            right_gripper = self.right_gripper_state if self.right_gripper_state is not None else self.get_gripper_state('right')
            
            # Left arm always has flexible gripper
            self.recorded_positions.append({
                'arm': 'left',
                'angles': left_angles,
                'waist': waist,
                'gripper': left_gripper,
                'gripper_type': 'flexible',
                'aux': aux.copy(),
                'timestamp': timestamp,
                'time_from_start': time_from_start,
                'delay': 5.0
            })
            
            # Right arm has parallel or vacuum
            self.recorded_positions.append({
                'arm': 'right',
                'angles': right_angles,
                'waist': waist,
                'gripper': right_gripper,
                'gripper_type': self.right_gripper_type,
                'aux': aux.copy(),
                'timestamp': timestamp,
                'time_from_start': time_from_start,
                'delay': 5.0
            })
            
            left_str = ", ".join([f"{a:6.1f}°" for a in left_angles])
            right_str = ", ".join([f"{a:6.1f}°" for a in right_angles])
            right_gripper_label = "Vacuum" if self.right_gripper_type == RIGHT_GRIPPER_VACUUM else "Gripper"
            print(f"✓ Position {len(self.recorded_positions)//2:3d} (both arms)")
            print(f"  LEFT:  [{left_str}] Gripper: {left_gripper:3d}")
            print(f"  RIGHT: [{right_str}] {right_gripper_label}: {right_gripper:3d}")
            print(f"  Waist: {waist:6.1f}° | AUX: [{aux_str}]\n")
        else:
            print(f"  → Reading {arm} arm position...")
            angles = self.read_position(arm)
            
            if arm == 'left':
                gripper = self.left_gripper_state if self.left_gripper_state is not None else self.get_gripper_state('left')
                gripper_type = 'flexible'
            else:
                gripper = self.right_gripper_state if self.right_gripper_state is not None else self.get_gripper_state('right')
                gripper_type = self.right_gripper_type
            
            self.recorded_positions.append({
                'arm': arm,
                'angles': angles,
                'waist': waist,
                'gripper': gripper,
                'gripper_type': gripper_type,
                'aux': aux.copy(),
                'timestamp': timestamp,
                'time_from_start': time_from_start,
                'delay': 5.0
            })
            
            angles_str = ", ".join([f"{a:6.1f}°" for a in angles])
            gripper_label = "Vacuum" if (arm == 'right' and self.right_gripper_type == RIGHT_GRIPPER_VACUUM) else "Gripper"
            print(f"✓ Position {len(self.recorded_positions):3d} | "
                  f"Waist: {waist:6.1f}° | {gripper_label}: {gripper:3d} | AUX: [{aux_str}]")
            print(f"  [{angles_str}]\n")
    
    def save_to_json(self, filename=None):
        if not self.recorded_positions:
            print("No positions to save")
            return
        
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.json"
        
        data = {
            'date': datetime.now().isoformat(),
            'total': len(self.recorded_positions),
            'has_gripper_data': True,
            'has_delay_data': True,
            'has_aux_data': True,
            'gripper_config': {
                'left': 'flexible',
                'right': self.right_gripper_type
            },
            'aux_pins': {
                'aux1': AUX1_PIN,
                'aux2': AUX2_PIN,
                'aux3': AUX3_PIN,
                'aux4': AUX4_PIN,
                'aux5': AUX5_PIN
            },
            'positions': self.recorded_positions
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"✓ Saved: {filename}")
        print(f"  Right gripper type: {self.right_gripper_type}")
        print(f"  Includes AUX data: Yes")
    
    def save_to_csv(self, filename=None):
        if not self.recorded_positions:
            print("No positions to save")
            return

        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.csv"

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['type', 'arm', 'waist', 'gripper', 'gripper_type', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'delay', 'aux1', 'aux2', 'aux3', 'aux4', 'aux5', 'teensy_ch'])

            for i, pos in enumerate(self.recorded_positions):
                # Handle Teensy type positions
                if pos.get('type') == 'teensy':
                    row = ['teensy', '', '', '', '', '', '', '', '', '', '',
                           f"{pos.get('delay', 0.5):.2f}", '', '', '', '', '', pos.get('channel', 1)]
                    writer.writerow(row)
                    continue

                if i + 1 < len(self.recorded_positions):
                    time_diff = self.recorded_positions[i+1]['time_from_start'] - pos['time_from_start']
                    default_delay = max(5.0, time_diff) if time_diff > 0.01 else 5.0
                else:
                    default_delay = 5.0

                aux = pos.get('aux', [0, 0, 0, 0, 0])

                row = [
                    'position',
                    pos['arm'],
                    f"{pos.get('waist', 0.0):.2f}",
                    pos.get('gripper', 0),
                    pos.get('gripper_type', 'parallel' if pos['arm'] == 'left' else 'flexible')
                ] + [f"{a:.2f}" for a in pos['angles']] + [
                    f"{pos.get('delay', default_delay):.2f}"
                ] + aux + ['']
                writer.writerow(row)

        print(f"✓ Saved: {filename}")
        print(f"  Left gripper type: {self.right_gripper_type}")
        print(f"  Includes AUX columns: aux1, aux2, aux3, aux4, aux5")
    
    def load_from_json(self, filename):
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.recorded_positions = data['positions']
            
            gripper_config = data.get('gripper_config', {})
            loaded_left_type = gripper_config.get('left', 'parallel')
            
            if loaded_left_type != self.right_gripper_type:
                print(f"⚠ Warning: Recording was made with LEFT gripper type: {loaded_left_type}")
                print(f"   Current LEFT gripper type: {self.right_gripper_type}")
            
            has_aux = data.get('has_aux_data', False)
            if not has_aux:
                print("⚠ Warning: Recording has no AUX data (adding default values)")
                for pos in self.recorded_positions:
                    if 'aux' not in pos:
                        pos['aux'] = [0, 0, 0, 0, 0]
            
            has_gripper = data.get('has_gripper_data', False)
            if not has_gripper:
                print("⚠ Warning: Recording has no gripper data (adding default values)")
                for pos in self.recorded_positions:
                    if 'gripper' not in pos:
                        pos['gripper'] = 0
            
            has_delay = data.get('has_delay_data', False)
            if not has_delay:
                print("⚠ Warning: Recording has no delay data (adding default values)")
                for pos in self.recorded_positions:
                    if 'delay' not in pos:
                        pos['delay'] = 5.0
            
            print(f"✓ Loaded: {len(self.recorded_positions)} positions from {filename}")
            
        except FileNotFoundError:
            print(f"❌ File not found: {filename}")
        except json.JSONDecodeError:
            print(f"❌ Invalid JSON: {filename}")
    
    def load_from_csv(self, filename):
        try:
            self.recorded_positions = []
            
            print(f"\nLoading CSV: {filename}")
            
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                
                required_columns = ['arm', 'waist', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6']
                if not all(col in reader.fieldnames for col in required_columns):
                    print(f"❌ Invalid CSV format. Required columns: {required_columns}")
                    return
                
                has_delay_column = 'delay' in reader.fieldnames
                has_gripper_column = 'gripper' in reader.fieldnames
                has_gripper_type_column = 'gripper_type' in reader.fieldnames
                has_aux_columns = all(f'aux{i}' in reader.fieldnames for i in range(1, 6))
                
                if has_delay_column:
                    print("✓ CSV has 'delay' column")
                else:
                    print("⚠ CSV has no 'delay' column - using default 5.0s")
                
                if has_gripper_column:
                    print("✓ CSV has 'gripper' column")
                else:
                    print("⚠ CSV has no 'gripper' column")
                
                if has_aux_columns:
                    print("✓ CSV has AUX columns (aux1-aux5)")
                else:
                    print("⚠ CSV has no AUX columns - using defaults")
                
                first_time = None
                left_count = 0
                right_count = 0
                teensy_count = 0

                has_type_column = 'type' in reader.fieldnames
                has_teensy_ch_column = 'teensy_ch' in reader.fieldnames

                for i, row in enumerate(reader):
                    try:
                        # Check if this is a Teensy action row
                        if has_type_column and row.get('type', '').strip().lower() == 'teensy':
                            channel = int(row.get('teensy_ch', 1)) if has_teensy_ch_column else 1
                            delay = float(row['delay']) if has_delay_column and row.get('delay') else 0.5

                            if first_time is None:
                                first_time = time.time()

                            self.recorded_positions.append({
                                'type': 'teensy',
                                'channel': channel,
                                'delay': delay,
                                'timestamp': first_time,
                                'time_from_start': 0
                            })
                            teensy_count += 1
                            continue

                        angles = [float(row[f'j{j}']) for j in range(1, 7)]

                        arm = row['arm'].strip().lower()
                        waist = float(row['waist'])
                        gripper = int(float(row['gripper'])) if has_gripper_column else 0

                        if has_gripper_type_column:
                            gripper_type = row['gripper_type'].strip().lower()
                        else:
                            gripper_type = 'vacuum' if (arm == 'left' and self.right_gripper_type == RIGHT_GRIPPER_VACUUM) else ('parallel' if arm == 'left' else 'flexible')

                        delay = float(row['delay']) if has_delay_column else 5.0

                        if has_aux_columns:
                            aux = [int(row[f'aux{j}']) for j in range(1, 6)]
                        else:
                            aux = [0, 0, 0, 0, 0]

                        time_value = float(row.get('time', 0))

                        if first_time is None:
                            first_time = time.time()
                        timestamp = first_time + time_value

                        self.recorded_positions.append({
                            'arm': arm,
                            'angles': angles,
                            'waist': waist,
                            'gripper': gripper,
                            'gripper_type': gripper_type,
                            'aux': aux,
                            'delay': delay,
                            'timestamp': timestamp,
                            'time_from_start': time_value
                        })

                        if arm == 'left':
                            left_count += 1
                        elif arm == 'right':
                            right_count += 1

                    except (ValueError, KeyError) as e:
                        print(f"⚠ Warning: Skipping row {i+2} due to error: {e}")
                        continue
            
            total_positions = len(self.recorded_positions)
            print(f"\n{'='*70}")
            print(f"✓ CSV LOADED SUCCESSFULLY")
            print(f"{'='*70}")
            print(f"File: {filename}")
            print(f"Total positions: {total_positions}")
            print(f"  Left arm:  {left_count} positions")
            print(f"  Right arm: {right_count} positions")
            if teensy_count > 0:
                print(f"  Teensy:    {teensy_count} actions")

            total_delay = sum(pos.get('delay', 5.0) for pos in self.recorded_positions)
            estimated_time = total_delay + (total_positions * 3)
            print(f"Estimated replay time: {estimated_time:.1f} seconds ({estimated_time/60:.1f} minutes)")
            print(f"{'='*70}\n")
            
        except FileNotFoundError:
            print(f"❌ File not found: {filename}")
        except Exception as e:
            print(f"❌ Error loading CSV: {e}")
    
    # === REPLAY ===
    def replay(self, speed=50, loop=False):
        if not self.recorded_positions:
            print("No recording to replay")
            return
        
        print(f"\n▶️  REPLAY - {len(self.recorded_positions)} positions")
        print(f"   Speed: {speed} | Loop: {loop}")
        print(f"   Using 'delay' values from recording\n")
        
        self.mybuddy.focus_servo(3, 1)
        time.sleep(0.1)
        self.mybuddy.power_on(3)
        time.sleep(0.3)
        
        self.set_led_color(*LED_RED)
        
        try:
            iteration = 1
            while True:
                if loop:
                    print(f"\n{'='*70}")
                    print(f"Iteration {iteration}")
                    print(f"{'='*70}\n")
                
                replay_start_time = time.time()
                
                i = 0
                while i < len(self.recorded_positions):
                    pos_start_time = time.time()

                    pos = self.recorded_positions[i]

                    # Handle Teensy actions
                    if pos.get('type') == 'teensy':
                        channel = pos.get('channel', 1)
                        elapsed = time.time() - replay_start_time
                        print(f"[{elapsed:6.1f}s] Position {i+1}/{len(self.recorded_positions)} | TEENSY CH{channel}")

                        # Send START command and wait for completion
                        if self.teensy_start_channel(channel):
                            result = self.teensy_wait_complete(channel)
                            if result == "COMPLETE":
                                print(f"{'':11s} ✓ CH{channel} completed successfully")
                            elif result == "FAILED":
                                print(f"{'':11s} ✗ CH{channel} failed - continuing sequence")
                            elif result == "TIMEOUT":
                                print(f"{'':11s} ⚠ CH{channel} timed out - continuing sequence")
                            else:
                                print(f"{'':11s} → CH{channel} skipped (no Teensy)")
                        else:
                            print(f"{'':11s} ⚠ Failed to send START command")

                        i += 1
                        continue

                    arm_id = 1 if pos['arm'] == 'left' else 2
                    arm_name = pos['arm'].upper()
                    waist = pos.get('waist', 0.0)
                    gripper = pos.get('gripper', 0)
                    gripper_type = pos.get('gripper_type', 'parallel' if pos['arm'] == 'left' else 'flexible')
                    delay = pos.get('delay', 0.5)
                    aux = pos.get('aux', [0, 0, 0, 0, 0])
                    
                    elapsed = time.time() - replay_start_time
                    
                    angles_str = ", ".join([f"{a:6.1f}°" for a in pos['angles']])
                    gripper_label = "Vacuum" if gripper_type == 'vacuum' else "Gripper"
                    aux_str = "".join([str(a) for a in aux])
                    
                    print(f"[{elapsed:6.1f}s] Position {i+1}/{len(self.recorded_positions)} | {arm_name:5s} | "
                          f"Waist: {waist:6.1f}° | {gripper_label}: {gripper:3d} | AUX: [{aux_str}]")
                    print(f"{'':11s} Angles: [{angles_str}]")
                    
                    # Set auxiliary outputs
                    for j, state in enumerate(aux):
                        if self.aux_states[j] != state:
                            self.set_aux(j + 1, state)
                    
                    # Move waist FIRST
                    try:
                        self.mybuddy.send_angle(3, 1, waist, speed)
                        time.sleep(0.1)
                    except Exception as e:
                        print(f"{'':11s} ⚠ Waist error: {e}")
                    
                    # Move arm
                    self.mybuddy.send_angles(arm_id, pos['angles'], speed)
                    time.sleep(delay)  # Wait for arm to reach position

                    # Debug: Check actual position reached
                    actual_angles = self.mybuddy.get_angles(arm_id)
                    if actual_angles:
                        diffs = [abs(a - t) for a, t in zip(actual_angles, pos['angles'])]
                        max_diff = max(diffs)
                        if max_diff > 5:
                            print(f"{'':11s} ⚠ Position error: max diff = {max_diff:.1f}°")
                            actual_str = ", ".join([f"{a:6.1f}°" for a in actual_angles])
                            print(f"{'':11s}   Actual: [{actual_str}]")

                    # Set gripper
                    if gripper_type == 'vacuum':
                        if gripper >= 50:
                            GPIO.output(VACUUM_PIN, GPIO.HIGH)
                            print(f"{'':11s} → Vacuum: ON")
                        else:
                            GPIO.output(VACUUM_PIN, GPIO.LOW)
                            print(f"{'':11s} → Vacuum: OFF")
                        time.sleep(0.5)
                    else:
                        try:
                            gripper_before = self.mybuddy.get_gripper_value(arm_id)
                            self.mybuddy.set_gripper_value(arm_id, gripper, GRIPPER_SPEED)
                            
                            if arm_id == 1:
                                print(f"{'':11s} → Left gripper: {gripper_before} → {gripper} (target)")
                            
                            time.sleep(2.0)
                            
                            gripper_after = self.mybuddy.get_gripper_value(arm_id)
                            
                            if arm_id == 1:
                                if abs(gripper_after - gripper) > 10:
                                    print(f"{'':11s} ⚠ Left gripper: Expected {gripper}, got {gripper_after}")
                                else:
                                    print(f"{'':11s} ✓ Left gripper: {gripper_after} (reached target)")
                            
                        except Exception as e:
                            print(f"{'':11s} ⚠ Gripper error: {e}")
                    
                    # Check for dual arm
                    if i + 1 < len(self.recorded_positions):
                        next_pos = self.recorded_positions[i + 1]
                        same_time = abs(next_pos['time_from_start'] - pos['time_from_start']) < 0.01
                        
                        if same_time:
                            time.sleep(0.1)
                            
                            next_arm_id = 1 if next_pos['arm'] == 'left' else 2
                            next_arm_name = next_pos['arm'].upper()
                            next_angles_str = ", ".join([f"{a:6.1f}°" for a in next_pos['angles']])
                            next_waist = next_pos.get('waist', 0.0)
                            next_gripper = next_pos.get('gripper', 0)
                            next_gripper_type = next_pos.get('gripper_type', 'parallel' if next_pos['arm'] == 'left' else 'flexible')
                            next_delay = next_pos.get('delay', 0.5)
                            next_aux = next_pos.get('aux', [0, 0, 0, 0, 0])
                            
                            next_gripper_label = "Vacuum" if next_gripper_type == 'vacuum' else "Gripper"
                            next_aux_str = "".join([str(a) for a in next_aux])
                            
                            elapsed = time.time() - replay_start_time
                            print(f"[{elapsed:6.1f}s]          + | {next_arm_name:5s} | "
                                  f"Waist: {next_waist:6.1f}° | {next_gripper_label}: {next_gripper:3d} | AUX: [{next_aux_str}]")
                            print(f"{'':11s} Angles: [{next_angles_str}]")
                            
                            # Set aux for second arm (should be same, but check)
                            for j, state in enumerate(next_aux):
                                if self.aux_states[j] != state:
                                    self.set_aux(j + 1, state)
                            
                            self.mybuddy.send_angles(next_arm_id, next_pos['angles'], speed)
                            
                            if next_gripper_type == 'vacuum':
                                if next_gripper >= 50:
                                    GPIO.output(VACUUM_PIN, GPIO.HIGH)
                                    print(f"{'':11s} → Vacuum: ON")
                                else:
                                    GPIO.output(VACUUM_PIN, GPIO.LOW)
                                    print(f"{'':11s} → Vacuum: OFF")
                                time.sleep(0.5)
                            else:
                                try:
                                    gripper_before = self.mybuddy.get_gripper_value(next_arm_id)
                                    self.mybuddy.set_gripper_value(next_arm_id, next_gripper, GRIPPER_SPEED)
                                    
                                    if next_arm_id == 1:
                                        print(f"{'':11s} → Left gripper: {gripper_before} → {next_gripper} (target)")
                                    
                                    time.sleep(2.0)
                                    
                                    gripper_after = self.mybuddy.get_gripper_value(next_arm_id)
                                    
                                    if next_arm_id == 1:
                                        if abs(gripper_after - next_gripper) > 10:
                                            print(f"{'':11s} ⚠ Left gripper: Expected {next_gripper}, got {gripper_after}")
                                        else:
                                            print(f"{'':11s} ✓ Left gripper: {gripper_after} (reached target)")
                                            
                                except Exception as e:
                                    print(f"{'':11s} ⚠ Gripper error: {e}")
                            
                            delay = next_delay
                            i += 1
                    
                    print(f"{'':11s} Pausing {delay:.1f}s...")
                    time.sleep(delay)
                    
                    pos_elapsed = time.time() - pos_start_time
                    print(f"{'':11s} (Position took {pos_elapsed:.1f}s total)\n")
                    
                    i += 1
                
                total_elapsed = time.time() - replay_start_time
                print(f"\n{'='*70}")
                print(f"✓ Replay complete - Total time: {total_elapsed:.1f} seconds")
                print(f"{'='*70}")
                
                if not loop:
                    break
                
                iteration += 1
                time.sleep(1)
        
        except KeyboardInterrupt:
            print("\n⏹ Stopped")
        finally:
            GPIO.output(VACUUM_PIN, GPIO.LOW)
            # Turn off all aux outputs on replay end
            for pin in AUX_PINS:
                GPIO.output(pin, GPIO.LOW)
            self.aux_states = [0, 0, 0, 0, 0]
            self.set_led_color(*LED_BLUE)
    
    # === MANUAL TEACHING ===
    def teach(self, arm):
        arm_name = "BOTH ARMS" if arm == 'both' else f"{arm.upper()} ARM"
        
        if arm == 'left':
            led_arm = 1
        elif arm == 'right':
            led_arm = 2
        else:
            led_arm = None
        
        print("\n" + "="*70)
        print(f"  MANUAL TEACHING MODE - {arm_name}")
        print("="*70)
        print("\nWorkflow:")
        print("  1. Hold GPIO 17 (Deadman) → Arm(s) become free")
        print("  2. Move arm(s) to desired position")
        print("  3. Release GPIO 17 → Arm(s) lock")  
        print("  4. Press GPIO 27 (Record) → Saves position")
        print("\nGripper Control:")
        print("  GPIO 22 - Toggle left gripper (flexible)")
        if self.right_gripper_type == RIGHT_GRIPPER_VACUUM:
            print("           Right arm uses vacuum (keyboard: von/voff)")
        else:
            print("           Right arm uses parallel (keyboard: ro/rc)")
        print("\nKeyboard commands (type and press ENTER):")
        print("  lo  - Open left gripper (flexible)")
        print("  lc  - Close left gripper (flexible)")
        if self.right_gripper_type == RIGHT_GRIPPER_VACUUM:
            print("  von - Turn vacuum ON (right arm)")
            print("  voff- Turn vacuum OFF (right arm)")
        else:
            print("  ro  - Open right gripper (parallel)")
            print("  rc  - Close right gripper (parallel)")
        print("\nAuxiliary Outputs:")
        print("  a1on / a1off - AUX1 (GPIO 8)")
        print("  a2on / a2off - AUX2 (GPIO 25)")
        print("  a3on / a3off - AUX3 (GPIO 24)")
        print("  a4on / a4off - AUX4 (GPIO 23)")
        print("  a5on / a5off - AUX5 (GPIO 18)")
        print("  aon  - All AUX ON")
        print("  aoff - All AUX OFF")
        print("  ax   - Show AUX status")
        print("\nWaist Control:")
        print("  w<angle> - Set waist (e.g., w45 or w-30)")
        print("  wf  - Free waist for manual movement")
        print("  wl  - Lock waist")
        print("\nTeensy Control (pump channels):")
        print("  t1  - Record Teensy CH1 start (wait for complete)")
        print("  t2  - Record Teensy CH2 start (wait for complete)")
        print("  t3  - Record Teensy CH3 start (wait for complete)")
        print("  t4  - Record Teensy CH4 start (wait for complete)")
        print("  ts  - Show Teensy status")
        print("\n  Ctrl+C - Finish")
        print("\n" + "="*70 + "\n")
        
        self.recorded_positions = []
        self.start_time = time.time()
        self.deadman_active = False

        import sys
        import select
        import termios
        import tty

        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setcbreak(sys.stdin.fileno())
            input_buffer = ""
            
            while True:
                if select.select([sys.stdin], [], [], 0)[0]:
                    char = sys.stdin.read(1)
                    
                    if char == '\n':
                        cmd = input_buffer.strip().lower()
                        
                        # Vacuum commands
                        if cmd == 'von':
                            self.vacuum_on()
                        elif cmd == 'voff':
                            self.vacuum_off()
                        
                        # Auxiliary output commands - explicit on/off
                        elif cmd == 'a1on':
                            self.set_aux(1, 1)
                        elif cmd == 'a1off':
                            self.set_aux(1, 0)
                        elif cmd == 'a2on':
                            self.set_aux(2, 1)
                        elif cmd == 'a2off':
                            self.set_aux(2, 0)
                        elif cmd == 'a3on':
                            self.set_aux(3, 1)
                        elif cmd == 'a3off':
                            self.set_aux(3, 0)
                        elif cmd == 'a4on':
                            self.set_aux(4, 1)
                        elif cmd == 'a4off':
                            self.set_aux(4, 0)
                        elif cmd == 'a5on':
                            self.set_aux(5, 1)
                        elif cmd == 'a5off':
                            self.set_aux(5, 0)
                        elif cmd == 'aon':
                            print("  All AUX ON:")
                            for i in range(1, 6):
                                self.set_aux(i, 1)
                        elif cmd == 'aoff':
                            print("  All AUX OFF:")
                            for i in range(1, 6):
                                self.set_aux(i, 0)
                        elif cmd == 'ax':
                            self.show_aux_status()
                        
                        # Gripper commands
                        # Left arm always has flexible gripper
                        elif cmd == 'lo':
                            self.open_gripper('left')
                        elif cmd == 'lc':
                            self.close_gripper('left')
                        # Right arm has parallel or vacuum
                        elif cmd == 'ro':
                            if self.right_gripper_type == RIGHT_GRIPPER_VACUUM:
                                self.vacuum_on()
                            else:
                                self.open_gripper('right')
                        elif cmd == 'rc':
                            if self.right_gripper_type == RIGHT_GRIPPER_VACUUM:
                                self.vacuum_off()
                            else:
                                self.close_gripper('right')
                        elif cmd == 'bo':
                            self.open_gripper('both')
                        elif cmd == 'bc':
                            self.close_gripper('both')
                        
                        # Waist commands
                        elif cmd.startswith('w'):
                            if cmd == 'wf':
                                self.release_waist()
                                print("\n🔓 Waist released - move manually\n")
                            elif cmd == 'wl':
                                self.lock_waist()
                                waist = self.read_waist()
                                print(f"\n🔒 Waist locked at {waist:6.1f}°\n")
                            else:
                                try:
                                    angle = float(cmd[1:])
                                    self.set_waist(angle)
                                except ValueError:
                                    print("\n⚠ Invalid. Use: w45 or wf or wl\n")

                        # Teensy commands
                        elif cmd == 't1':
                            self.record_teensy_action(1)
                        elif cmd == 't2':
                            self.record_teensy_action(2)
                        elif cmd == 't3':
                            self.record_teensy_action(3)
                        elif cmd == 't4':
                            self.record_teensy_action(4)
                        elif cmd == 'ts':
                            self.teensy_read_status()
                            with self.teensy_lock:
                                if self.teensy_status:
                                    print(f"\n  Teensy Status:")
                                    print(f"    Setpoint: {self.teensy_status.get('setpoint', '?')} mL")
                                    for ch_idx, ch in enumerate(self.teensy_status.get('channels', [])):
                                        print(f"    CH{ch_idx+1}: {ch.get('state', '?')} - {ch.get('ml', 0):.1f} mL")
                                    print()
                                else:
                                    print("\n  ⚠ No Teensy status available\n")

                        input_buffer = ""
                    elif char == '\x03':
                        raise KeyboardInterrupt
                    else:
                        input_buffer += char
                
                # Check deadman
                if GPIO.input(DEADMAN_PIN) == GPIO.LOW:
                    if not self.deadman_active:
                        self.release_arm(arm)
                        self.deadman_active = True
                        self.set_led_color(*LED_GREEN, arm=led_arm)
                        arm_text = "Arms are" if arm == 'both' else "Arm is"
                        print(f"🟢 DEADMAN ACTIVE - {arm_text} free to move\n")
                else:
                    if self.deadman_active:
                        self.lock_arm(arm)
                        self.deadman_active = False
                        self.set_led_color(*LED_BLUE, arm=led_arm)
                        arm_text = "Arms locked" if arm == 'both' else "Arm locked"
                        print(f"🔵 DEADMAN RELEASED - {arm_text}")
                        print("   Press GPIO 27 to record this position\n")
                    
                    # Check record button
                    if GPIO.input(RECORD_PIN) == GPIO.LOW:
                        current_time = time.time()
                        if current_time - self.last_record_time > self.record_debounce:
                            self.set_led_color(*LED_YELLOW, arm=led_arm)
                            time.sleep(0.15)
                            self.set_led_color(*LED_BLUE, arm=led_arm)
                            
                            self.record_position(arm)
                            self.last_record_time = current_time
                    
                    # Check left gripper button (GPIO 22) - always controls left (flexible) gripper
                    if GPIO.input(LEFT_GRIPPER_PIN) == GPIO.LOW:
                        current_time = time.time()
                        if current_time - self.last_left_gripper_time > self.gripper_debounce:
                            if arm in ['left', 'both']:
                                self.toggle_gripper('left')
                            self.last_left_gripper_time = current_time
                
                time.sleep(0.05)
        
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            if self.deadman_active:
                self.lock_arm(arm)
            GPIO.output(VACUUM_PIN, GPIO.LOW)
            # Turn off all aux outputs
            for pin in AUX_PINS:
                GPIO.output(pin, GPIO.LOW)
            self.aux_states = [0, 0, 0, 0, 0]
            self.set_led_color(*LED_BLUE, arm=led_arm)
            print(f"\n✓ Session complete: {len(self.recorded_positions)} positions\n")


def main():
    print("="*70)
    print("  MyBuddy 2 Training & Replay System v200_R")
    print("  Left: Flexible Gripper | Right: Parallel OR Vacuum")
    print("  ✓ AUX OUTPUTS: a1on/a1off, a2on/a2off, etc.")
    print("="*70)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='MyBuddy 2 Training & Replay System')
    parser.add_argument('--gripper', '-g', choices=['parallel', 'vacuum'], default='vacuum',
                        help='Right arm gripper type (default: vacuum)')
    args = parser.parse_args()

    if args.gripper == 'vacuum':
        right_gripper_type = RIGHT_GRIPPER_VACUUM
        print("✓ Right arm: VACUUM GRIPPER selected")
    else:
        right_gripper_type = RIGHT_GRIPPER_PARALLEL
        print("✓ Right arm: PARALLEL GRIPPER selected")
    
    print()
    
    try:
        trainer = MyBuddyTrainer(right_gripper_type=right_gripper_type)
    except Exception as e:
        print(f"❌ Initialization failed: {e}")
        return
    
    try:
        while True:
            print("\n--- MENU ---")
            print("1. Record LEFT arm")
            print("2. Record RIGHT arm")
            print("3. Record BOTH arms (synchronized)")
            print("4. Save recording (JSON)")
            print("5. Save recording (CSV)")
            print("6. Load recording (JSON)")
            print("7. Load recording (CSV)")
            print("8. Replay")
            print("9. Gripper/AUX Test Menu")
            print("h. Go to HOME position")
            print("s. Set current as HOME")
            print("0. Exit")
            
            choice = input("\nSelect: ").strip()
            
            if choice == '1':
                trainer.teach('left')
            
            elif choice == '2':
                trainer.teach('right')
            
            elif choice == '3':
                trainer.teach('both')
            
            elif choice == '4':
                filename = input("Filename (or ENTER for auto): ").strip()
                trainer.save_to_json(filename if filename else None)
            
            elif choice == '5':
                filename = input("CSV filename (or ENTER for auto): ").strip()
                trainer.save_to_csv(filename if filename else None)
            
            elif choice == '6':
                filename = input("JSON filename: ").strip()
                trainer.load_from_json(filename)
            
            elif choice == '7':
                filename = input("CSV filename: ").strip()
                trainer.load_from_csv(filename)
            
            elif choice == '8':
                speed = int(input("Speed (1-100, default 50): ") or "50")
                loop = input("Loop? (y/n): ").lower() == 'y'
                trainer.replay(speed, loop)
            
            elif choice == '9':
                print("\n--- GRIPPER/AUX TEST ---")
                print("1. Open left gripper (flexible)")
                print("2. Close left gripper (flexible)")
                if trainer.right_gripper_type == RIGHT_GRIPPER_VACUUM:
                    print("3. Turn vacuum ON (right arm)")
                    print("4. Turn vacuum OFF (right arm)")
                else:
                    print("3. Open right gripper (parallel)")
                    print("4. Close right gripper (parallel)")
                print("--- AUX Outputs ---")
                print("51. AUX1 ON    50. AUX1 OFF")
                print("61. AUX2 ON    60. AUX2 OFF")
                print("71. AUX3 ON    70. AUX3 OFF")
                print("81. AUX4 ON    80. AUX4 OFF")
                print("91. AUX5 ON    90. AUX5 OFF")
                print("aa. All AUX ON")
                print("ao. All AUX OFF")
                print("s.  Show AUX status")
                print("0.  Back to main menu")
                
                test_choice = input("\nSelect: ").strip().lower()
                
                if test_choice == '1':
                    trainer.open_gripper('left')
                elif test_choice == '2':
                    trainer.close_gripper('left')
                elif test_choice == '3':
                    if trainer.right_gripper_type == RIGHT_GRIPPER_VACUUM:
                        trainer.vacuum_on()
                    else:
                        trainer.open_gripper('right')
                elif test_choice == '4':
                    if trainer.right_gripper_type == RIGHT_GRIPPER_VACUUM:
                        trainer.vacuum_off()
                    else:
                        trainer.close_gripper('right')
                elif test_choice == '51':
                    trainer.set_aux(1, 1)
                elif test_choice == '50':
                    trainer.set_aux(1, 0)
                elif test_choice == '61':
                    trainer.set_aux(2, 1)
                elif test_choice == '60':
                    trainer.set_aux(2, 0)
                elif test_choice == '71':
                    trainer.set_aux(3, 1)
                elif test_choice == '70':
                    trainer.set_aux(3, 0)
                elif test_choice == '81':
                    trainer.set_aux(4, 1)
                elif test_choice == '80':
                    trainer.set_aux(4, 0)
                elif test_choice == '91':
                    trainer.set_aux(5, 1)
                elif test_choice == '90':
                    trainer.set_aux(5, 0)
                elif test_choice == 'aa':
                    print("  All AUX ON:")
                    for i in range(1, 6):
                        trainer.set_aux(i, 1)
                elif test_choice == 'ao':
                    print("  All AUX OFF:")
                    for i in range(1, 6):
                        trainer.set_aux(i, 0)
                elif test_choice == 's':
                    trainer.show_aux_status()

            elif choice == 'h':
                trainer.go_home()

            elif choice == 's':
                trainer.set_home()

            elif choice == '0':
                break
    
    finally:
        trainer.cleanup()
        print("Goodbye!")


if __name__ == "__main__":
    main()
