"""
LIDAR 2D->3D Viewer

Expect serial lines (CSV):
  <servo_deg>,<scan_deg>,<distance_mm>\n
Example line:
  30,90,1200    # servo pitch 30°, lidar scan angle 90°, distance 1200 mm

Notes/assumptions:
- LIDAR produces 2D scans where `scan_deg` is the azimuth in the LIDAR plane
  (0° forward, increasing counter-clockwise). Distance is in millimeters.
- `servo_deg` is the tilt/pitch angle applied to the entire LIDAR (degrees).
- Coordinate mapping used:
    local LIDAR point before tilt: x = r*cos(theta), y = r*sin(theta), z = 0
    apply rotation about Y (pitch = servo_deg):
      X' = cos(phi)*x
      Y' = y
      Z' = -sin(phi)*x
  (This is a common convention; flip sign/axes with CLI flags if needed.)

Usage:
  python scripts/lidar_3d_viewer.py --port COM8 --baud 115200

Dependencies:
  pip install pyserial numpy
  optional for nicer rendering: pip install open3d
  fallback visualization uses matplotlib: pip install matplotlib

If you can't change your STM firmware, you can adapt the `parse_line` function
below to fit whatever CSV/format you output.

Microcontroller example (pseudo-C) to send lines:
  // send servo angle, scan angle, distance(mm) as CSV
  printf("%d,%d,%d\n", servo_angle_deg, scan_angle_deg, distance_mm);

"""

import argparse
import threading
import time
from collections import deque

import numpy as np

try:
    import serial
except Exception as e:
    raise RuntimeError("pyserial required: pip install pyserial")

# Optional visualization libs
try:
    import open3d as o3d
    _has_o3d = True
except Exception:
    _has_o3d = False

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    _has_mpl = True
except Exception:
    _has_mpl = False

DEFAULT_MAX_POINTS = 200000


def parse_line(line: str):
    """Parse a CSV line. Return (servo_deg, scan_deg, dist_mm) or None on failure."""
    line = line.strip()
    if not line:
        return None
    # Allow comments
    if line.startswith('#'):
        return None
    # Accept comma separated fields
    parts = line.split(',')
    if len(parts) < 3:
        return None
    try:
        servo = float(parts[0])
        scan = float(parts[1])
        dist = float(parts[2])
        return servo, scan, dist
    except Exception:
        return None


def polar_to_xyz(r_mm, theta_deg, phi_deg):
    # convert to meters
    r = r_mm / 1000.0
    th = np.deg2rad(theta_deg)
    ph = np.deg2rad(phi_deg)
    x = r * np.cos(th)
    y = r * np.sin(th)
    # rotate about Y by phi: [cos 0 sin; 0 1 0; -sin 0 cos]
    X = np.cos(ph) * x + 0 * y
    Y = y
    Z = -np.sin(ph) * x + 0 * y
    return X, Y, Z


class SerialReader(threading.Thread):
    def __init__(self, port, baud, queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.queue = queue
        self.stop_event = stop_event
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            print(f"Failed to open serial port {self.port}: {e}")
            self.stop_event.set()
            return

        print(f"Serial opened {self.port} @ {self.baud}")
        while not self.stop_event.is_set():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore')
            except Exception:
                time.sleep(0.01)
                continue
            parsed = parse_line(line)
            if parsed is None:
                continue
            servo, scan, dist = parsed
            x, y, z = polar_to_xyz(dist, scan, servo)
            self.queue.append((x, y, z))
            # throttle queue size
            if len(self.queue) > DEFAULT_MAX_POINTS:
                for _ in range(len(self.queue) - DEFAULT_MAX_POINTS):
                    self.queue.popleft()

        if self.ser and self.ser.is_open:
            self.ser.close()


def run_open3d(queue, stop_event, update_interval=0.05):
    pcd = o3d.geometry.PointCloud()
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='LIDAR 3D View')
    vis.add_geometry(pcd)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    while not stop_event.is_set():
        if len(queue) > 0:
            pts = np.asarray(queue)
            pcd.points = o3d.utility.Vector3dVector(pts)
            vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(update_interval)
    vis.destroy_window()


def run_matplotlib(queue, stop_event, update_interval=0.1):
    plt.ion()
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    sc = None
    while not stop_event.is_set():
        if len(queue) > 0:
            pts = np.asarray(queue)
            ax.clear()
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=1)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            # autoscale
            mn = pts.min(axis=0)
            mx = pts.max(axis=0)
            ax.set_xlim(mn[0], mx[0])
            ax.set_ylim(mn[1], mx[1])
            ax.set_zlim(mn[2], mx[2])
            plt.pause(0.001)
        time.sleep(update_interval)
    plt.close(fig)


def run_simulator(queue, stop_event):
    # Simple simulator for testing without hardware
    servo_angles = np.linspace(-45, 45, 21)
    scan_angles = np.linspace(0, 360, 181)
    t = 0
    while not stop_event.is_set():
        phi = float(servo_angles[int(t) % len(servo_angles)])
        for th in scan_angles:
            # simple test pattern: circle radius varies with angle
            r = 1000 + 200 * np.sin(np.deg2rad(th * 3 + t))
            x, y, z = polar_to_xyz(r, th, phi)
            queue.append((x, y, z))
            if len(queue) > DEFAULT_MAX_POINTS:
                queue.popleft()
        t += 1
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description='2D LIDAR + servo -> 3D viewer')
    parser.add_argument('--port', type=str, default=None, help='Serial port (COMx or /dev/ttyX)')
    parser.add_argument('--baud', type=int, default=115200, help='Serial baud rate')
    parser.add_argument('--simulate', action='store_true', help='Run simulator instead of serial')
    parser.add_argument('--no-open3d', action='store_true', help='Disable Open3D even if installed')
    args = parser.parse_args()

    points = deque()
    stop_event = threading.Event()

    if args.simulate:
        reader = threading.Thread(target=run_simulator, args=(points, stop_event), daemon=True)
        reader.start()
    else:
        if not args.port:
            print('Error: --port required unless --simulate is set')
            return
        reader = SerialReader(args.port, args.baud, points, stop_event)
        reader.start()

    try:
        if _has_o3d and not args.no_open3d:
            run_open3d(points, stop_event)
        elif _has_mpl:
            run_matplotlib(points, stop_event)
        else:
            print('No visualization library found. Install open3d or matplotlib.')
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        time.sleep(0.2)


if __name__ == '__main__':
    main()
