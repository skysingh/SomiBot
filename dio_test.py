#!/usr/bin/env python3
"""
MyBuddy M5ATOM Digital IO Test Utility

Pins: 1-5 on each arm (maps internally to G22/G19/G23/G33)
Arm IDs: 1 = Left, 2 = Right

API:
  set_tool_pin_mode(id, pin_no, pin_mode)  - mode: 0=input, 1=output
  set_tool_digital_output(id, pin_no, signal) - signal: 0=LOW, 1=HIGH
  get_tool_digital_input(id, pin_no) - returns 0 or 1
"""

from pymycobot import MyBuddy
import time
import sys

PORT = '/dev/ttyACM0'
BAUD = 115200

# M5ATOM IO pins (API uses 1-5, maps to G22/G19/G23/G33 internally)
DIO_PINS = [1, 2, 3, 4, 5]

def connect():
    print(f"Connecting to MyBuddy on {PORT}...")
    try:
        mb = MyBuddy(PORT, BAUD)
        time.sleep(2)
        print("Connected!")

        # Verify M5ATOM communication with LED flash
        print("Verifying M5ATOM communication...")
        print("  LEFT arm LED -> RED")
        mb.set_tool_color(1, 255, 0, 0)
        time.sleep(0.5)
        print("  RIGHT arm LED -> RED")
        mb.set_tool_color(2, 255, 0, 0)
        time.sleep(0.5)
        print("  Both LEDs -> GREEN")
        mb.set_tool_color(1, 0, 255, 0)
        mb.set_tool_color(2, 0, 255, 0)
        time.sleep(0.5)
        print("  Both LEDs -> BLUE")
        mb.set_tool_color(1, 0, 0, 255)
        mb.set_tool_color(2, 0, 0, 255)
        time.sleep(0.5)
        print("  Both LEDs -> OFF")
        mb.set_tool_color(1, 0, 0, 0)
        mb.set_tool_color(2, 0, 0, 0)
        print("M5ATOM communication OK!\n")
        return mb
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

def test_led(mb, arm_id):
    """Test LED on specified arm"""
    arm_name = "LEFT" if arm_id == 1 else "RIGHT"
    print(f"\n=== {arm_name} ARM LED TEST ===")

    colors = [
        ("RED", 255, 0, 0),
        ("GREEN", 0, 255, 0),
        ("BLUE", 0, 0, 255),
        ("YELLOW", 255, 255, 0),
        ("CYAN", 0, 255, 255),
        ("MAGENTA", 255, 0, 255),
        ("WHITE", 255, 255, 255),
    ]

    for name, r, g, b in colors:
        print(f"  {name}...")
        mb.set_tool_color(arm_id, r, g, b)
        time.sleep(1)

    print("  OFF")
    mb.set_tool_color(arm_id, 0, 0, 0)
    print("Done")

def test_output_single(mb, arm_id, pin):
    """Test single pin as output"""
    arm_name = "LEFT" if arm_id == 1 else "RIGHT"
    print(f"\n[{arm_name} ARM] Pin {pin} OUTPUT Test")
    print("-" * 35)

    # Set as output
    print(f"  set_tool_pin_mode({arm_id}, {pin}, 1)...", end=" ", flush=True)
    try:
        mb.set_tool_pin_mode(arm_id, pin, 1)
        time.sleep(0.2)
        print("OK")
    except Exception as e:
        print(f"FAILED: {e}")
        return

    # Set HIGH
    print(f"  set_tool_digital_output({arm_id}, {pin}, 1)...", end=" ", flush=True)
    try:
        mb.set_tool_digital_output(arm_id, pin, 1)
        print("HIGH - measure 3.3V for 3 sec")
        time.sleep(3)
    except Exception as e:
        print(f"FAILED: {e}")

    # Set LOW
    print(f"  set_tool_digital_output({arm_id}, {pin}, 0)...", end=" ", flush=True)
    try:
        mb.set_tool_digital_output(arm_id, pin, 0)
        print("LOW")
        time.sleep(0.5)
    except Exception as e:
        print(f"FAILED: {e}")

def test_input_single(mb, arm_id, pin):
    """Test single pin as input"""
    arm_name = "LEFT" if arm_id == 1 else "RIGHT"
    print(f"\n[{arm_name} ARM] Pin {pin} INPUT Test")
    print("-" * 35)

    # Set as input
    print(f"  set_tool_pin_mode({arm_id}, {pin}, 0)...", end=" ", flush=True)
    try:
        mb.set_tool_pin_mode(arm_id, pin, 0)
        time.sleep(0.2)
        print("OK")
    except Exception as e:
        print(f"FAILED: {e}")
        return

    # Read 5 times
    print(f"  Reading pin (connect to 3.3V or GND):")
    for i in range(5):
        try:
            val = mb.get_tool_digital_input(arm_id, pin)
            state = "HIGH" if val else "LOW"
            print(f"    Read {i+1}: {val} ({state})")
            time.sleep(0.5)
        except Exception as e:
            print(f"    Read {i+1}: FAILED - {e}")

def test_all_outputs(mb, arm_id):
    """Test all pins as outputs on one arm"""
    arm_name = "LEFT" if arm_id == 1 else "RIGHT"
    print(f"\n{'='*50}")
    print(f"  {arm_name} ARM - ALL PINS OUTPUT TEST")
    print(f"{'='*50}")

    for pin in DIO_PINS:
        test_output_single(mb, arm_id, pin)

def test_all_inputs(mb, arm_id):
    """Test all pins as inputs on one arm"""
    arm_name = "LEFT" if arm_id == 1 else "RIGHT"
    print(f"\n{'='*50}")
    print(f"  {arm_name} ARM - ALL PINS INPUT TEST")
    print(f"{'='*50}")

    for pin in DIO_PINS:
        test_input_single(mb, arm_id, pin)

def test_output_blink(mb, arm_id, pin, count=5):
    """Blink a pin for visual test"""
    arm_name = "LEFT" if arm_id == 1 else "RIGHT"
    print(f"\n[{arm_name} ARM] Pin {pin} BLINK Test ({count} times)")
    print("-" * 35)

    # Set as output
    try:
        mb.set_tool_pin_mode(arm_id, pin, 1)
        time.sleep(0.1)
    except Exception as e:
        print(f"  Failed to set output mode: {e}")
        return

    print(f"  Blinking...")
    for i in range(count):
        try:
            mb.set_tool_digital_output(arm_id, pin, 1)
            time.sleep(0.3)
            mb.set_tool_digital_output(arm_id, pin, 0)
            time.sleep(0.3)
            print(f"    Blink {i+1}")
        except Exception as e:
            print(f"    Blink {i+1}: FAILED - {e}")
    print("  Done")

def test_output_hold(mb, arm_id, pin, seconds=30):
    """Hold pin HIGH for measurement"""
    arm_name = "LEFT" if arm_id == 1 else "RIGHT"
    print(f"\n[{arm_name} ARM] Pin {pin} HOLD HIGH for {seconds}s")
    print("-" * 35)

    # Set as output
    try:
        mb.set_tool_pin_mode(arm_id, pin, 1)
        time.sleep(0.1)
    except Exception as e:
        print(f"  Failed to set output mode: {e}")
        return

    # Set HIGH and hold
    try:
        mb.set_tool_digital_output(arm_id, pin, 1)
        print(f"  Pin HIGH - measure voltage now...")
        for i in range(seconds, 0, -5):
            print(f"    {i} seconds remaining...")
            time.sleep(5)
        mb.set_tool_digital_output(arm_id, pin, 0)
        print("  Pin LOW - Done")
    except Exception as e:
        print(f"  Error: {e}")

def interactive_mode(mb):
    """Interactive DIO control"""
    print("\n" + "="*50)
    print("  INTERACTIVE DIO CONTROL")
    print("="*50)
    print("\nCommands:")
    print("  out <arm> <pin>      - Set pin as output (arm: 1/2, pin: 1-5)")
    print("  in <arm> <pin>       - Set pin as input")
    print("  high <arm> <pin>     - Set output HIGH")
    print("  low <arm> <pin>      - Set output LOW")
    print("  read <arm> <pin>     - Read input")
    print("  blink <arm> <pin>    - Blink output 5 times")
    print("  status               - Show all pin states")
    print("  quit                 - Exit")
    print()

    while True:
        try:
            cmd = input("DIO> ").strip().lower()
        except EOFError:
            break

        if cmd == 'quit' or cmd == 'q':
            break

        if cmd == 'status':
            print("\n  Reading all pins...")
            for arm_id in [1, 2]:
                arm_name = "LEFT" if arm_id == 1 else "RIGHT"
                print(f"  {arm_name} ARM:")
                for pin in DIO_PINS:
                    try:
                        val = mb.get_tool_digital_input(arm_id, pin)
                        print(f"    Pin {pin}: {val}")
                    except:
                        print(f"    Pin {pin}: ERROR")
            print()
            continue

        parts = cmd.split()
        if len(parts) < 3:
            print("  Invalid command")
            continue

        action = parts[0]
        try:
            arm_id = int(parts[1])
            pin = int(parts[2])
        except ValueError:
            print("  Invalid arm/pin number")
            continue

        if arm_id not in [1, 2]:
            print("  Arm must be 1 (left) or 2 (right)")
            continue
        if pin not in DIO_PINS:
            print(f"  Pin must be 1-5")
            continue

        arm_name = "LEFT" if arm_id == 1 else "RIGHT"

        try:
            if action == 'out':
                mb.set_tool_pin_mode(arm_id, pin, 1)
                print(f"  {arm_name} Pin {pin}: OUTPUT mode")
            elif action == 'in':
                mb.set_tool_pin_mode(arm_id, pin, 0)
                print(f"  {arm_name} Pin {pin}: INPUT mode")
            elif action == 'high':
                mb.set_tool_digital_output(arm_id, pin, 1)
                print(f"  {arm_name} Pin {pin}: HIGH")
            elif action == 'low':
                mb.set_tool_digital_output(arm_id, pin, 0)
                print(f"  {arm_name} Pin {pin}: LOW")
            elif action == 'read':
                val = mb.get_tool_digital_input(arm_id, pin)
                state = "HIGH" if val else "LOW"
                print(f"  {arm_name} Pin {pin}: {val} ({state})")
            elif action == 'blink':
                mb.set_tool_pin_mode(arm_id, pin, 1)
                for _ in range(5):
                    mb.set_tool_digital_output(arm_id, pin, 1)
                    time.sleep(0.3)
                    mb.set_tool_digital_output(arm_id, pin, 0)
                    time.sleep(0.3)
                print(f"  {arm_name} Pin {pin}: Blinked 5 times")
            else:
                print("  Unknown action")
        except Exception as e:
            print(f"  Error: {e}")

def main():
    print("="*50)
    print("  MyBuddy M5ATOM Digital IO Test")
    print("  Arm 1 = LEFT | Arm 2 = RIGHT")
    print("  Pins: 1-5 (maps to G22/G19/G23/G33)")
    print("="*50)

    mb = connect()

    while True:
        print("\n--- DIO Test Menu ---")
        print("LED Tests (verify M5ATOM communication):")
        print("  L1. LED test - LEFT arm")
        print("  L2. LED test - RIGHT arm")
        print("  L3. LED test - BOTH arms")
        print()
        print("OUTPUT Tests:")
        print("  1. Test ALL outputs - LEFT arm")
        print("  2. Test ALL outputs - RIGHT arm")
        print("  3. Test SINGLE output (select pin)")
        print("  4. Blink test (select arm/pin)")
        print("  5. Hold HIGH test (30s for measurement)")
        print()
        print("INPUT Tests:")
        print("  6. Test ALL inputs - LEFT arm")
        print("  7. Test ALL inputs - RIGHT arm")
        print("  8. Test SINGLE input (select pin)")
        print()
        print("  9. Interactive mode")
        print("  0. Exit")

        choice = input("\nSelect [0-9]: ").strip()

        if choice.lower() == 'l1':
            test_led(mb, 1)
        elif choice.lower() == 'l2':
            test_led(mb, 2)
        elif choice.lower() == 'l3':
            test_led(mb, 1)
            test_led(mb, 2)
        elif choice == '1':
            test_all_outputs(mb, 1)
        elif choice == '2':
            test_all_outputs(mb, 2)
        elif choice == '3':
            arm = input("  Arm (1=left, 2=right): ").strip()
            pin = input("  Pin (1-5): ").strip()
            try:
                test_output_single(mb, int(arm), int(pin))
            except ValueError:
                print("  Invalid input")
        elif choice == '4':
            arm = input("  Arm (1=left, 2=right): ").strip()
            pin = input("  Pin (1-5): ").strip()
            try:
                test_output_blink(mb, int(arm), int(pin))
            except ValueError:
                print("  Invalid input")
        elif choice == '5':
            arm = input("  Arm (1=left, 2=right): ").strip()
            pin = input("  Pin (1-5): ").strip()
            try:
                test_output_hold(mb, int(arm), int(pin))
            except ValueError:
                print("  Invalid input")
        elif choice == '6':
            test_all_inputs(mb, 1)
        elif choice == '7':
            test_all_inputs(mb, 2)
        elif choice == '8':
            arm = input("  Arm (1=left, 2=right): ").strip()
            pin = input("  Pin (1-5): ").strip()
            try:
                test_input_single(mb, int(arm), int(pin))
            except ValueError:
                print("  Invalid input")
        elif choice == '9':
            interactive_mode(mb)
        elif choice == '0':
            print("\nGoodbye!")
            break
        else:
            print("Invalid choice")

if __name__ == "__main__":
    main()
