import serial
import time

# Serial port configuration
SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200
BYTESIZE = serial.EIGHTBITS      # 8 data bits
PARITY = serial.PARITY_ODD       # Odd parity
STOPBITS = serial.STOPBITS_TWO   # 2 stop bits
TIMEOUT = 1                      # 1 second read timeout

# Example: we expect an 8-byte request from the other device
EXPECTED_REQUEST_SIZE = 8

# This is the *larger* reply you want to send. Replace the hex data below with your actual reply.
# For example, let's assume it's 40 bytes (just an example). Make sure to match your actual data length.
# Example: "EB 90 06 25 01 5A F0 F0 7B 7B 00 7A C1 70 12 09 ... 00 88"
RESPONSE_HEX = (
    "EB 90 06 25 01 5A F0 F0 "  
    "7B 7B 00 7A C1 70 12 09 "  # Additional bytes
    "FF FF AA 55 12 34 56 78 "  # More data
    "00 00 00 00 00 01 15 7D "  # More data
    "FF FF AA 55 12 34 56 78 "                     # Example ending
    "EB 90 06 25 01 5A F0 F0 "  
    "7B 7B 00 7A C1 70 12 09 "  # Additional bytes
    "FF FF AA 55 12 34 56 78 "  # More data
    "00 00 00 00 00 01 15 7D "  # More data
    "FF FF AA 55 12 34 56 78 "
    "EB 90 06 25 01 5A F0 F0 "  
    "7B 7B 00 7A C1 70 12 09 "  # Additional bytes
    "FF FF AA 55 12 34 56 78 "  # More data
    "00 00 00 00 00 01 15 7D "  # More data
    "FF FF AA 55 12 34 56 78 "
    "FF FF AA 55 12 34 56 78 "
)
RESPONSE = bytes.fromhex(RESPONSE_HEX)

def main():
    # Open the serial port with the specified settings
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        bytesize=BYTESIZE,
        parity=PARITY,
        stopbits=STOPBITS,
        timeout=TIMEOUT
    )
    print(f"Listening on {SERIAL_PORT} at {BAUDRATE} baud.")
    print(f"Expecting a {EXPECTED_REQUEST_SIZE}-byte request and then sending a large reply.")

    try:
        while True:
            # Check if data is waiting in the receive buffer
            while ser.in_waiting:
                # Mark the start time
                start_time = time.perf_counter()

                # Read the expected number of bytes (or read until the full request is received)
                request_data = ser.read(EXPECTED_REQUEST_SIZE)
                # print(f"Received TX command: {request_data.hex().upper()}")

                # Immediately send the larger reply
                ser.write(RESPONSE)
                # ser.flush()  # Ensure data is handed off to the driver

                # Mark the end time
                end_time = time.perf_counter()
                elapsed_ms = (end_time - start_time) * 1000
                print(f"Sent reply ({len(RESPONSE)} bytes): {RESPONSE.hex().upper()}")
                print(f"Round-trip timing: {elapsed_ms:.3f} ms\n")

    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
