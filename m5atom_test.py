#!/usr/bin/env python3
"""
M5ATOM Controller Test Utility
Tests M5ATOM controllers on each arm of MyBuddy independently

Arm IDs:
  1 = Left arm
  2 = Right arm
"""

from pymycobot import MyBuddy
import time
import sys

# Connection settings
PORT = '/dev/ttyACM0'
BAUD = 115200

# Test colors
COLORS = {
    'red':    (255, 0, 0),
    'green':  (0, 255, 0),
    'blue':   (0, 0, 255),
    'yellow': (255, 255, 0),
    'cyan':   (0, 255, 255),
    'magenta':(255, 0, 255),
    'white':  (255, 255, 255),
    'off':    (0, 0, 0)
}

def connect():
    """Connect to MyBuddy robot"""
    print(f"Connecting to MyBuddy on {PORT}...")
    try:
        mb = MyBuddy(PORT, BAUD)
        time.sleep(2)
        print("  Connected!\n")
        return mb
    except Exception as e:
        print(f"  ERROR: {e}")
        sys.exit(1)

def test_led(mb, arm_id, arm_name):
    """Test LED on specified arm's M5ATOM"""
    print(f"\n=== Testing {arm_name} Arm LED (Arm {arm_id}) ===")
    print("Cycling through colors...")

    for color_name, (r, g, b) in COLORS.items():
        if color_name == 'off':
            continue
        print(f"  {color_name.upper()}...", end=" ", flush=True)
        try:
            mb.set_tool_color(arm_id, r, g, b)
            time.sleep(1)
            print("OK")
        except Exception as e:
            print(f"FAILED: {e}")

    # Turn off
    mb.set_tool_color(arm_id, 0, 0, 0)
    print(f"  LED OFF")

def test_gripper_read(mb, arm_id, arm_name):
    """Test reading gripper value"""
    print(f"\n=== Testing {arm_name} Arm Gripper Read (Arm {arm_id}) ===")

    for i in range(3):
        try:
            value = mb.get_gripper_value(arm_id)
            print(f"  Read {i+1}: Gripper value = {value}")
            time.sleep(0.5)
        except Exception as e:
            print(f"  Read {i+1}: FAILED - {e}")

    return value

def test_gripper_write(mb, arm_id, arm_name):
    """Test writing gripper value"""
    print(f"\n=== Testing {arm_name} Arm Gripper Write (Arm {arm_id}) ===")

    # Open gripper
    print("  Opening gripper (value=100)...", end=" ", flush=True)
    try:
        mb.set_gripper_value(arm_id, 100, 50)
        time.sleep(2)
        val = mb.get_gripper_value(arm_id)
        print(f"OK (read back: {val})")
    except Exception as e:
        print(f"FAILED: {e}")

    # Close gripper
    print("  Closing gripper (value=0)...", end=" ", flush=True)
    try:
        mb.set_gripper_value(arm_id, 0, 50)
        time.sleep(2)
        val = mb.get_gripper_value(arm_id)
        print(f"OK (read back: {val})")
    except Exception as e:
        print(f"FAILED: {e}")

    # Mid position
    print("  Mid position (value=50)...", end=" ", flush=True)
    try:
        mb.set_gripper_value(arm_id, 50, 50)
        time.sleep(2)
        val = mb.get_gripper_value(arm_id)
        print(f"OK (read back: {val})")
    except Exception as e:
        print(f"FAILED: {e}")

def test_arm_full(mb, arm_id, arm_name):
    """Run all tests on one arm"""
    print("\n" + "="*50)
    print(f"  FULL TEST: {arm_name.upper()} ARM (ID={arm_id})")
    print("="*50)

    test_led(mb, arm_id, arm_name)
    test_gripper_read(mb, arm_id, arm_name)
    test_gripper_write(mb, arm_id, arm_name)

def led_hold_test(mb, arm_id, arm_name):
    """Hold LED on for visual inspection"""
    print(f"\n=== LED Hold Test: {arm_name} Arm ===")
    print("LED will stay on for 30 seconds each color.")
    print("Watch the M5ATOM on the arm.\n")

    for color_name in ['red', 'green', 'blue']:
        r, g, b = COLORS[color_name]
        print(f"  {color_name.upper()} - holding for 30 seconds...")
        mb.set_tool_color(arm_id, r, g, b)
        time.sleep(30)

    mb.set_tool_color(arm_id, 0, 0, 0)
    print("  OFF")

def interactive_led(mb):
    """Interactive LED control"""
    print("\n=== Interactive LED Control ===")
    print("Commands:")
    print("  <arm> <color>  - Set color (e.g., 'left red', 'right blue')")
    print("  both <color>   - Set both arms")
    print("  off            - Turn off both")
    print("  quit           - Exit")
    print(f"\nColors: {', '.join(COLORS.keys())}")
    print()

    while True:
        cmd = input("> ").strip().lower()

        if cmd == 'quit':
            break
        elif cmd == 'off':
            mb.set_tool_color(1, 0, 0, 0)
            mb.set_tool_color(2, 0, 0, 0)
            print("  Both LEDs OFF")
        else:
            parts = cmd.split()
            if len(parts) == 2:
                arm, color = parts
                if color in COLORS:
                    r, g, b = COLORS[color]
                    if arm == 'left':
                        mb.set_tool_color(1, r, g, b)
                        print(f"  Left arm: {color}")
                    elif arm == 'right':
                        mb.set_tool_color(2, r, g, b)
                        print(f"  Right arm: {color}")
                    elif arm == 'both':
                        mb.set_tool_color(1, r, g, b)
                        mb.set_tool_color(2, r, g, b)
                        print(f"  Both arms: {color}")
                    else:
                        print("  Invalid arm. Use: left, right, both")
                else:
                    print(f"  Invalid color. Use: {', '.join(COLORS.keys())}")
            else:
                print("  Invalid command")

# ============== DIO TESTS ==============

# M5ATOM DIO pins (typically pins 23, 33 on M5ATOM)
TOOL_DIO_PINS = [23, 33]

def test_dio_output(mb, arm_id, arm_name):
    """Test digital outputs on M5ATOM"""
    print(f"\n=== Testing {arm_name} Arm DIO Outputs (Arm {arm_id}) ===")
    print(f"Testing pins: {TOOL_DIO_PINS}")
    print()

    for pin in TOOL_DIO_PINS:
        print(f"  Pin {pin}:")

        # Set pin as output
        print(f"    Setting pin mode to OUTPUT...", end=" ", flush=True)
        try:
            mb.set_tool_pin_mode(arm_id, pin, 0)  # 0 = output mode
            time.sleep(0.2)
            print("OK")
        except Exception as e:
            print(f"FAILED: {e}")
            continue

        # Set HIGH
        print(f"    Setting HIGH...", end=" ", flush=True)
        try:
            mb.set_tool_digital_output(arm_id, pin, 1)
            time.sleep(1)
            print("OK (check with multimeter: ~3.3V)")
        except Exception as e:
            print(f"FAILED: {e}")

        # Set LOW
        print(f"    Setting LOW...", end=" ", flush=True)
        try:
            mb.set_tool_digital_output(arm_id, pin, 0)
            time.sleep(0.5)
            print("OK (should be ~0V)")
        except Exception as e:
            print(f"FAILED: {e}")

        print()

def test_dio_input(mb, arm_id, arm_name):
    """Test digital inputs on M5ATOM"""
    print(f"\n=== Testing {arm_name} Arm DIO Inputs (Arm {arm_id}) ===")
    print(f"Testing pins: {TOOL_DIO_PINS}")
    print("Connect pin to 3.3V for HIGH, GND for LOW")
    print()

    for pin in TOOL_DIO_PINS:
        print(f"  Pin {pin}:")

        # Set pin as input
        print(f"    Setting pin mode to INPUT...", end=" ", flush=True)
        try:
            mb.set_tool_pin_mode(arm_id, pin, 1)  # 1 = input mode
            time.sleep(0.2)
            print("OK")
        except Exception as e:
            print(f"FAILED: {e}")
            continue

        # Read value multiple times
        print(f"    Reading values (3 samples):")
        for i in range(3):
            try:
                val = mb.get_tool_digital_input(arm_id, pin)
                state = "HIGH" if val else "LOW"
                print(f"      Read {i+1}: {val} ({state})")
                time.sleep(0.5)
            except Exception as e:
                print(f"      Read {i+1}: FAILED - {e}")

        print()

def test_dio_blink(mb, arm_id, arm_name):
    """Blink DIO outputs for visual test"""
    print(f"\n=== DIO Blink Test: {arm_name} Arm (Arm {arm_id}) ===")
    print(f"Blinking pins {TOOL_DIO_PINS} - connect LEDs to see output")
    print()

    for pin in TOOL_DIO_PINS:
        # Set as output
        try:
            mb.set_tool_pin_mode(arm_id, pin, 0)
        except:
            print(f"  Pin {pin}: Failed to set output mode")
            continue

        print(f"  Pin {pin}: Blinking 5 times...")
        for _ in range(5):
            try:
                mb.set_tool_digital_output(arm_id, pin, 1)
                time.sleep(0.3)
                mb.set_tool_digital_output(arm_id, pin, 0)
                time.sleep(0.3)
            except Exception as e:
                print(f"    Error: {e}")
                break
        print(f"  Pin {pin}: Done")

    print()

def test_dio_hold(mb, arm_id, arm_name):
    """Hold DIO outputs HIGH for measurement"""
    print(f"\n=== DIO Hold Test: {arm_name} Arm (Arm {arm_id}) ===")
    print(f"Pins {TOOL_DIO_PINS} will be held HIGH for 30 seconds each")
    print("Use multimeter to verify 3.3V output")
    print()

    for pin in TOOL_DIO_PINS:
        # Set as output
        try:
            mb.set_tool_pin_mode(arm_id, pin, 0)
        except:
            print(f"  Pin {pin}: Failed to set output mode")
            continue

        print(f"  Pin {pin}: Setting HIGH, holding 30 seconds...")
        try:
            mb.set_tool_digital_output(arm_id, pin, 1)
            time.sleep(30)
            mb.set_tool_digital_output(arm_id, pin, 0)
            print(f"  Pin {pin}: Done (now LOW)")
        except Exception as e:
            print(f"  Pin {pin}: Error - {e}")

    print()

def interactive_dio(mb):
    """Interactive DIO control"""
    print("\n=== Interactive DIO Control ===")
    print("Commands:")
    print("  <arm> <pin> out       - Set pin as output")
    print("  <arm> <pin> in        - Set pin as input")
    print("  <arm> <pin> high      - Set output HIGH")
    print("  <arm> <pin> low       - Set output LOW")
    print("  <arm> <pin> read      - Read input value")
    print("  quit                  - Exit")
    print()
    print("  arm: left, right")
    print(f"  pin: {TOOL_DIO_PINS}")
    print()

    while True:
        cmd = input("DIO> ").strip().lower()

        if cmd == 'quit':
            break

        parts = cmd.split()
        if len(parts) != 3:
            print("  Invalid command. Use: <arm> <pin> <command>")
            continue

        arm_str, pin_str, action = parts

        # Parse arm
        if arm_str == 'left':
            arm_id = 1
        elif arm_str == 'right':
            arm_id = 2
        else:
            print("  Invalid arm. Use: left, right")
            continue

        # Parse pin
        try:
            pin = int(pin_str)
        except ValueError:
            print(f"  Invalid pin. Use: {TOOL_DIO_PINS}")
            continue

        # Execute action
        try:
            if action == 'out':
                mb.set_tool_pin_mode(arm_id, pin, 0)
                print(f"  {arm_str} pin {pin}: OUTPUT mode")
            elif action == 'in':
                mb.set_tool_pin_mode(arm_id, pin, 1)
                print(f"  {arm_str} pin {pin}: INPUT mode")
            elif action == 'high':
                mb.set_tool_digital_output(arm_id, pin, 1)
                print(f"  {arm_str} pin {pin}: HIGH")
            elif action == 'low':
                mb.set_tool_digital_output(arm_id, pin, 0)
                print(f"  {arm_str} pin {pin}: LOW")
            elif action == 'read':
                val = mb.get_tool_digital_input(arm_id, pin)
                state = "HIGH" if val else "LOW"
                print(f"  {arm_str} pin {pin}: {val} ({state})")
            else:
                print("  Invalid action. Use: out, in, high, low, read")
        except Exception as e:
            print(f"  Error: {e}")

def main():
    print("="*50)
    print("  M5ATOM Controller Test Utility")
    print("  MyBuddy Cobot - Arm 1 (Left) / Arm 2 (Right)")
    print("="*50)

    mb = connect()

    while True:
        print("\n--- M5ATOM Test Menu ---")
        print("=== LED & Gripper ===")
        print("1. Test LEFT arm (LED + Gripper)")
        print("2. Test RIGHT arm (LED + Gripper)")
        print("3. Test BOTH arms")
        print("4. Interactive LED control")
        print()
        print("=== DIO Tests ===")
        print("5. DIO Output test - LEFT arm")
        print("6. DIO Output test - RIGHT arm")
        print("7. DIO Input test - LEFT arm")
        print("8. DIO Input test - RIGHT arm")
        print("9. DIO Blink test - LEFT arm")
        print("10. DIO Blink test - RIGHT arm")
        print("11. DIO Hold test - LEFT (30s HIGH)")
        print("12. DIO Hold test - RIGHT (30s HIGH)")
        print("13. Interactive DIO control")
        print()
        print("0. Exit")

        choice = input("\nSelect: ").strip()

        if choice == '1':
            test_arm_full(mb, 1, "Left")
        elif choice == '2':
            test_arm_full(mb, 2, "Right")
        elif choice == '3':
            test_arm_full(mb, 1, "Left")
            test_arm_full(mb, 2, "Right")
        elif choice == '4':
            interactive_led(mb)
        elif choice == '5':
            test_dio_output(mb, 1, "Left")
        elif choice == '6':
            test_dio_output(mb, 2, "Right")
        elif choice == '7':
            test_dio_input(mb, 1, "Left")
        elif choice == '8':
            test_dio_input(mb, 2, "Right")
        elif choice == '9':
            test_dio_blink(mb, 1, "Left")
        elif choice == '10':
            test_dio_blink(mb, 2, "Right")
        elif choice == '11':
            test_dio_hold(mb, 1, "Left")
        elif choice == '12':
            test_dio_hold(mb, 2, "Right")
        elif choice == '13':
            interactive_dio(mb)
        elif choice == '0':
            mb.set_tool_color(1, 0, 0, 0)
            mb.set_tool_color(2, 0, 0, 0)
            print("\nGoodbye!")
            break
        else:
            print("Invalid choice")

if __name__ == "__main__":
    main()
