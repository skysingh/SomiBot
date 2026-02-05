#!/usr/bin/env python3
"""
RIGHT ARM ONLY - M5ATOM DIO Test
Pins 1-5 on Arm ID 2
"""

from pymycobot import MyBuddy
import time

PORT = '/dev/ttyACM0'
BAUD = 115200

print("="*40)
print("  RIGHT ARM DIO TEST")
print("  Arm ID: 2 | Pins: 1-5")
print("="*40)

print("\nConnecting to MyBuddy...")
mb = MyBuddy(PORT, BAUD)
time.sleep(2)
print("Connected!\n")

while True:
    print("--- Menu ---")
    print("1. LED test (red/green/blue)")
    print("2. Blink ALL pins (1-5)")
    print("3. Blink single pin")
    print("4. Hold pin HIGH (30s)")
    print("5. Read all pins")
    print("0. Exit")

    choice = input("\nSelect: ").strip()

    if choice == '0':
        mb.set_tool_color(2, 0, 0, 0)
        print("Bye!")
        break

    elif choice == '1':
        print("\n  RED...")
        mb.set_tool_color(2, 255, 0, 0)
        time.sleep(2)
        print("  GREEN...")
        mb.set_tool_color(2, 0, 255, 0)
        time.sleep(2)
        print("  BLUE...")
        mb.set_tool_color(2, 0, 0, 255)
        time.sleep(2)
        print("  OFF")
        mb.set_tool_color(2, 0, 0, 0)

    elif choice == '2':
        print("\n  Blinking ALL pins 5 times...")
        for pin in [1, 2, 3, 4, 5]:
            mb.set_tool_pin_mode(2, pin, 1)
            time.sleep(0.1)

        for i in range(5):
            print(f"    Cycle {i+1}: HIGH")
            for pin in [1, 2, 3, 4, 5]:
                mb.set_tool_digital_output(2, pin, 1)
            time.sleep(0.5)

            print(f"    Cycle {i+1}: LOW")
            for pin in [1, 2, 3, 4, 5]:
                mb.set_tool_digital_output(2, pin, 0)
            time.sleep(0.5)
        print("  Done")

    elif choice == '3':
        pin = input("  Pin number (1-5): ").strip()
        try:
            pin = int(pin)
            if pin < 1 or pin > 5:
                print("  Invalid pin")
                continue
        except:
            print("  Invalid pin")
            continue

        print(f"\n  Setting pin {pin} as output...")
        mb.set_tool_pin_mode(2, pin, 1)
        time.sleep(0.2)

        print(f"  Blinking pin {pin} - 10 times...")
        for i in range(10):
            mb.set_tool_digital_output(2, pin, 1)
            print(f"    {i+1}: HIGH")
            time.sleep(0.5)
            mb.set_tool_digital_output(2, pin, 0)
            print(f"    {i+1}: LOW")
            time.sleep(0.5)
        print("  Done")

    elif choice == '4':
        pin = input("  Pin number (1-5): ").strip()
        try:
            pin = int(pin)
            if pin < 1 or pin > 5:
                print("  Invalid pin")
                continue
        except:
            print("  Invalid pin")
            continue

        print(f"\n  Setting pin {pin} as output...")
        mb.set_tool_pin_mode(2, pin, 1)
        time.sleep(0.2)

        print(f"  Pin {pin} = HIGH for 30 seconds")
        print("  Measure voltage now!")
        mb.set_tool_digital_output(2, pin, 1)

        for i in range(30, 0, -5):
            print(f"    {i}s remaining...")
            time.sleep(5)

        mb.set_tool_digital_output(2, pin, 0)
        print("  Pin LOW - Done")

    elif choice == '5':
        print("\n  Reading all pins...")
        for pin in [1, 2, 3, 4, 5]:
            mb.set_tool_pin_mode(2, pin, 0)  # input
            time.sleep(0.1)
            val = mb.get_tool_digital_input(2, pin)
            print(f"    Pin {pin}: {val}")

    else:
        print("  Invalid choice")

    print()
