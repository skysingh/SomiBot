#!/usr/bin/env python3
"""
GPIO Output Test Script
Test pins: 7, 8, 25, 24, 23, 18
Connect LED with 2k resistor between GPIO pin and GND
"""

import RPi.GPIO as GPIO
import time

OUTPUT_PINS = [7, 8, 25, 24, 23, 18]

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in OUTPUT_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    print("GPIO initialized. All pins set to LOW.")

def test_individual():
    """Test each pin one at a time"""
    print("\n=== Individual Pin Test ===")
    print("Move your LED to each pin to verify output.\n")

    for pin in OUTPUT_PINS:
        input(f"Press Enter to test GPIO {pin}...")
        GPIO.output(pin, GPIO.HIGH)
        print(f"  GPIO {pin} = HIGH (LED should be ON)")
        time.sleep(1)
        GPIO.output(pin, GPIO.LOW)
        print(f"  GPIO {pin} = LOW (LED should be OFF)\n")

def test_blink_all():
    """Blink all pins together"""
    print("\n=== Blink All Pins Test ===")
    print("All output pins will blink 5 times.\n")

    for i in range(5):
        for pin in OUTPUT_PINS:
            GPIO.output(pin, GPIO.HIGH)
        print(f"  Cycle {i+1}: ALL HIGH")
        time.sleep(0.5)

        for pin in OUTPUT_PINS:
            GPIO.output(pin, GPIO.LOW)
        print(f"  Cycle {i+1}: ALL LOW")
        time.sleep(0.5)

def test_sequence():
    """Light up pins in sequence"""
    print("\n=== Sequence Test ===")
    print("Pins will light up one after another.\n")

    for _ in range(2):
        for pin in OUTPUT_PINS:
            GPIO.output(pin, GPIO.HIGH)
            print(f"  GPIO {pin} ON")
            time.sleep(0.3)
            GPIO.output(pin, GPIO.LOW)

def manual_control():
    """Manual on/off control"""
    print("\n=== Manual Control ===")
    print("Commands: <pin> on | <pin> off | all on | all off | quit")
    print(f"Valid pins: {OUTPUT_PINS}\n")

    while True:
        cmd = input("> ").strip().lower()

        if cmd == "quit":
            break
        elif cmd == "all on":
            for pin in OUTPUT_PINS:
                GPIO.output(pin, GPIO.HIGH)
            print("All pins HIGH")
        elif cmd == "all off":
            for pin in OUTPUT_PINS:
                GPIO.output(pin, GPIO.LOW)
            print("All pins LOW")
        else:
            parts = cmd.split()
            if len(parts) == 2:
                try:
                    pin = int(parts[0])
                    state = parts[1]
                    if pin in OUTPUT_PINS and state in ["on", "off"]:
                        GPIO.output(pin, GPIO.HIGH if state == "on" else GPIO.LOW)
                        print(f"GPIO {pin} = {'HIGH' if state == 'on' else 'LOW'}")
                    else:
                        print(f"Invalid. Use: {OUTPUT_PINS} on/off")
                except ValueError:
                    print("Invalid command")
            else:
                print("Invalid command")

def main():
    try:
        setup()

        while True:
            print("\n" + "="*40)
            print("GPIO Output Test Menu")
            print("="*40)
            print("1. Test individual pins (one at a time)")
            print("2. Blink all pins together")
            print("3. Sequence test")
            print("4. Manual control")
            print("5. Exit")

            choice = input("\nSelect test [1-5]: ").strip()

            if choice == "1":
                test_individual()
            elif choice == "2":
                test_blink_all()
            elif choice == "3":
                test_sequence()
            elif choice == "4":
                manual_control()
            elif choice == "5":
                break
            else:
                print("Invalid choice")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up.")

if __name__ == "__main__":
    main()
