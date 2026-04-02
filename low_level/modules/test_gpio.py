from periphery import GPIO
import time

def gpio_output_with_feedback():
    # GPIO Configuration (modify pin numbers based on your hardware)
    # PK3 (output) → maps to pin 323 of /dev/gpiochip0
    # PK4 (input)  → maps to pin 324 of /dev/gpiochip0
    OUTPUT_PIN_CHIP = "/dev/gpiochip0"
    OUTPUT_PIN_NUMBER = 323  # PK3 (output pin, controlled by the script)
    INPUT_PIN_NUMBER = 324   # PK4 (input pin, reads PK3's output state)

    # Initialize GPIO objects as None first (for safe release later)
    gpio_out = None
    gpio_in = None

    try:
        # Initialize PK3 as OUTPUT mode
        gpio_out = GPIO(OUTPUT_PIN_CHIP, OUTPUT_PIN_NUMBER, "out")
        # Initialize PK4 as INPUT mode
        gpio_in = GPIO(OUTPUT_PIN_CHIP, INPUT_PIN_NUMBER, "in")

        # Print test initialization info
        print("=== GPIO Output-Input Feedback Test Started ===")
        print(f"Controlled Pin (PK3): {OUTPUT_PIN_CHIP} - Pin {OUTPUT_PIN_NUMBER} (OUTPUT)")
        print(f"Monitoring Pin (PK4): {OUTPUT_PIN_CHIP} - Pin {INPUT_PIN_NUMBER} (INPUT)")
        print("Test Behavior: PK3 toggles HIGH/LOW every 1s; PK4 verifies PK3's state")
        print("Press Ctrl+C to stop the test\n")

        # Main loop: Toggle PK3 and read PK4 feedback
        while True:
            # 1. Set PK3 to HIGH level
            gpio_out.write(True)
            time.sleep(0.1)  # Short delay for signal stabilization (avoid read lag)
            pk4_reading = gpio_in.read()
            print(f"PK3 Output: HIGH (True) | PK4 Reading: {pk4_reading}")

            # Keep PK3 HIGH for 1 second
            time.sleep(1)

            # 2. Set PK3 to LOW level
            gpio_out.write(False)
            time.sleep(0.1)  # Short delay for signal stabilization
            pk4_reading = gpio_in.read()
            print(f"PK3 Output: LOW (False) | PK4 Reading: {pk4_reading}")

            # Keep PK3 LOW for 1 second
            time.sleep(1)

    # Handle user-initiated exit (Ctrl+C)
    except KeyboardInterrupt:
        print("\n\nTest stopped by user (Ctrl+C)")
    # Handle other unexpected errors (e.g., GPIO access failure)
    except Exception as e:
        print(f"\nError during test: {str(e)}")
    # Ensure GPIO resources are released even if an error occurs
    finally:
        print("\nReleasing GPIO resources...")
        # Safely close PK3 (set to LOW first to avoid residual high level)
        if gpio_out:
            try:
                gpio_out.write(False)
                gpio_out.close()
                print(f"Successfully closed PK3 (Pin {OUTPUT_PIN_NUMBER})")
            except Exception as close_err:
                print(f"Failed to close PK3 (Pin {OUTPUT_PIN_NUMBER}): {str(close_err)}")
        # Safely close PK4
        if gpio_in:
            try:
                gpio_in.close()
                print(f"Successfully closed PK4 (Pin {INPUT_PIN_NUMBER})")
            except Exception as close_err:
                print(f"Failed to close PK4 (Pin {INPUT_PIN_NUMBER}): {str(close_err)}")
        print("Resource release complete.")

# Run the test when the script is executed directly
if __name__ == "__main__":
    gpio_output_with_feedback()