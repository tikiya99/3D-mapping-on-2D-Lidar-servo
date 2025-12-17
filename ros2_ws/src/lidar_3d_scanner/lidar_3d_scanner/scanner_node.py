#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import serial
import struct
import math
import sys

# Protocol: "P <servo_angle_deg> <dist_mm> <conf> <lidar_angle_deg>"

class Lidar3DScanner(Node):
    def __init__(self):
        super().__init__('scanner_node')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'map')
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.publisher_ = self.create_publisher(PointCloud2, 'lidar_3d_points', 10)
        
        try:
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Connected to STM32 on {port} at {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            sys.exit(1)

        self.points_buffer = []
        self.publish_timer = self.create_timer(0.1, self.publish_cloud) # 10Hz publish
        self.read_timer = self.create_timer(0.001, self.read_serial) # Fast read

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line.startswith('P '):
                    parts = line.split(' ')
                    if len(parts) == 5:
                        # P <servo> <dist> <conf> <angle>
                        servo_deg = float(parts[1])
                        dist_mm = float(parts[2])
                        conf = int(parts[3])
                        lidar_deg = float(parts[4])
                        
                        if dist_mm > 0:# and conf > 100: # Threshold confidence if needed
                            self.process_point(servo_deg, dist_mm, lidar_deg)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')

    def process_point(self, servo_deg, dist_mm, lidar_deg):
        # Convert to Cartesian
        # Servo rotates around Y axis? Or Z?
        # Let's assume:
        # Lidar is mounted horizontally. Servo rotates it up/down (Pitch) or left/right (Yaw).
        # Common setup: Lidar spins in XY plane. Servo tilts this plane.
        # Let's assume Servo rotates the Lidar around the Y axis (Pitch).
        # Servo 90 = Horizontal.
        # Servo 0 = Looking Down? Or Up? 
        # Lidar Angle 0 = Forward.
        
        # Spherical coords:
        # r = dist_mm / 1000.0
        # phi (servo) = servo_deg
        # theta (lidar) = lidar_deg
        
        r = dist_mm / 1000.0
        
        # Angles to radians
        phi_rad = math.radians(servo_deg - 90.0) # 0 is horizontal
        theta_rad = math.radians(lidar_deg)
        
        # Logic: 
        # If Servo is Pitching the disk:
        # projected_r = r * cos(phi)
        # z = r * sin(phi)
        # x = projected_r * cos(theta)
        # y = projected_r * sin(theta)
        
        # Adjust based on real mounting. 
        # Assuming: Servo at 90 deg -> Lidar flat.
        # Servo moves 0..180.
        
        # Simple Spherical projection
        # x = r * cos(theta) * cos(phi)
        # y = r * sin(theta) * cos(phi)
        # z = r * sin(phi)
        
        x = r * math.cos(theta_rad) * math.cos(phi_rad)
        y = r * math.sin(theta_rad) * math.cos(phi_rad)
        z = r * math.sin(phi_rad)
        
        self.points_buffer.append((x, y, z))

    def publish_cloud(self):
        if not self.points_buffer:
            return
            
        # Create PointCloud2 msg
        # We accumulate points and publish chunks. 
        # For a full map, we might want to accumulate more or use rviz decay time.
        
        # Standard PointCloud2 creation is verbose in Python.
        # Structure: x, y, z (float32)
        
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.height = 1
        msg.width = len(self.points_buffer)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        buffer = []
        for p in self.points_buffer:
            buffer.append(struct.pack('fff', p[0], p[1], p[2]))
            
        msg.data = b''.join(buffer)
        
        self.publisher_.publish(msg)
        self.points_buffer = [] # Clear buffer after publish

def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
