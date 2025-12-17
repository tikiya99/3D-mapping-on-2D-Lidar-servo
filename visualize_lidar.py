import serial
import serial.tools.list_ports
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
import queue

# --- Configuration ---
BATCH_SIZE = 200   # Number of points to accumulation before re-plotting
MAX_POINTS = 2000  # Keep only recent points for real-time visualization to maintain FPS
                   # (The full map is saved to file)

data_queue = queue.Queue()
is_running = True

def find_stm_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "STMicroelectronics" in p.manufacturer or "STM" in p.description:
            return p.device
    return None

def serial_reader(port):
    global is_running
    try:
        ser = serial.Serial(port, 230400, timeout=1)
        print(f"Connected to {port}. Logging to 'scan_data.xyz'...")
        
        with open("scan_data.xyz", "w") as f:
            while is_running:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("P "):
                        parts = line.split()
                        if len(parts) >= 5:
                            try:
                                # Parsing
                                servo_deg = float(parts[1])
                                dist = float(parts[2])
                                lidar_deg = float(parts[4])

                                # Math (Spherical -> Cartesian)
                                # Adjust servo: 90 is horizontal. 
                                elevation = np.radians(servo_deg - 90)
                                azimuth = np.radians(lidar_deg)

                                # x = r * cos(el) * cos(az)
                                # y = r * cos(el) * sin(az)
                                # z = r * sin(el)
                                x = dist * np.cos(elevation) * np.cos(azimuth)
                                y = dist * np.cos(elevation) * np.sin(azimuth)
                                z = dist * np.sin(elevation)

                                # Save to File (X Y Z)
                                f.write(f"{x:.2f} {y:.2f} {z:.2f}\n")

                                # Send to Visualizer
                                data_queue.put((x, y, z))

                            except ValueError:
                                pass
                else:
                    time.sleep(0.001)
        ser.close()
    except Exception as e:
        print(f"Serial Error: {e}")
        is_running = False

def main():
    global is_running
    print("--- STM32 Lidar 3D Mapper ---")
    print("1. Real-time view shows LAST 2000 points (for speed).")
    print("2. FULL cloud saved to 'scan_data.xyz'.")
    
    port = find_stm_port()
    if not port:
        port = input("Enter COM port: ").strip()

    # Start Serial Thread
    t = threading.Thread(target=serial_reader, args=(port,))
    t.daemon = True
    t.start()

    # Setup Plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_xlim(-1500, 1500)
    ax.set_ylim(-1500, 1500)
    ax.set_zlim(-1000, 1000)

    xs, ys, zs = [], [], []

    try:
        while is_running and plt.fignum_exists(fig.number):
            # Process Queue
            batch_count = 0
            while not data_queue.empty() and batch_count < BATCH_SIZE:
                pt = data_queue.get()
                xs.append(pt[0])
                ys.append(pt[1])
                zs.append(pt[2])
                batch_count += 1
            
            # Update Plot if we have new data
            if batch_count > 0:
                # Limit buffer size for performance
                if len(xs) > MAX_POINTS:
                    xs = xs[-MAX_POINTS:]
                    ys = ys[-MAX_POINTS:]
                    zs = zs[-MAX_POINTS:]
                
                ax.clear() # Clearing is faster than appending thousands of distinct scatter objects
                ax.set_xlim(-1500, 1500)
                ax.set_ylim(-1500, 1500)
                ax.set_zlim(-1000, 1000)
                ax.scatter(xs, ys, zs, c='b', marker='.', s=2)
                
                plt.pause(0.05) # 20 FPS cap
            else:
                plt.pause(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        t.join(timeout=1)
        print("\nDon. Saved full cloud to 'scan_data.xyz'.")

if __name__ == "__main__":
    main()
