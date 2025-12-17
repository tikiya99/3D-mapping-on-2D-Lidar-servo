import serial
import serial.tools.list_ports
import time
import sys

def find_stm_port():
    """Finds the STM32 Nucleo serial port automatically."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # STM32 Nucleo typically shows up as STMicroelectronics or similar
        if "STMicroelectronics" in p.manufacturer or "STM" in p.description:
            return p.device
    return None

def main():
    print("--- STM32 Lidar Validation Tool ---")
    
    # Allow manual port override
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        print("Scanning for STM32 board...")
        port = find_stm_port()
        if not port:
            print("Could not auto-detect STM32 board.")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f" - {p.device} ({p.description})")
            port = input("Please enter the COM port (e.g., COM3 or /dev/ttyACM0): ").strip()

    print(f"Connecting to {port} at 230400 baud...")
    
    try:
        ser = serial.Serial(port, 230400, timeout=1)
        print("Connected! Waiting for data...")
        print("(Press Ctrl+C to stop)")
        
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"RX: {line}")
                        if line.startswith("P "):
                            parts = line.split()
                            if len(parts) >= 5:
                                print(f" -> Valid Point: Servo={parts[1]}, Dist={parts[2]}, Angle={parts[4]}")
                except Exception as e:
                    print(f"Read Error: {e}")
            else:
                time.sleep(0.01)

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
