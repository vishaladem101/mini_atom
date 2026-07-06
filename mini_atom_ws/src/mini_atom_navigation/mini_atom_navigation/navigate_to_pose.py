#!/usr/bin/env python3
import os
try:
    from dotenv import load_dotenv, find_dotenv
    load_dotenv(find_dotenv())
except ImportError:
    pass

import rclpy
from rclpy.node import Node
import socket
import struct
import math
import sys
import select
import termios
import tty
import threading

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import TransformBroadcaster

class IntegratedRobotController(Node):
    def __init__(self):
        super().__init__('integrated_robot_controller')
        
        # 1. NETWORK CONFIGURATIONS
        self.ESP32_IP = os.environ.get("ESP32_IP", "10.42.0.127")   # Ensure this matches your ESP32's active IP
        self.ESP32_PORT = 8888
        self.LAPTOP_PORT = 9999

        # 2. ROBOT PHYSICAL GEOMETRY CALIBRATION
        self.WHEEL_DIAMETER = 0.044    
        self.WHEEL_SEPARATION = 0.104  
        self.TICKS_PER_REV = 1060.0
        self.TICKS_PER_METER = self.TICKS_PER_REV / (math.pi * self.WHEEL_DIAMETER)

        # Odometry Tracker States
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_th = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.first_packet = True

        # 3. CLOSED LOOP NAVIGATION CONTROLLER MATRIX
        self.goal_x = None
        self.goal_y = None
        self.navigation_active = False
        self.current_goal_cell_index = None # Keeps track of where the goal marker is drawn
        
        # Gain tuners for autonomous tracking
        self.Kp_linear = 0.5
        self.Kp_angular = 2.0
        self.max_linear_speed = 0.18   # m/s safety ceiling limit
        self.max_angular_speed = 1.5   # rad/s safety ceiling limit
        self.goal_tolerance = 0.05     # 5cm destination arrival threshold

        # To keep track of the last velocity sent (for encoder direction determination)
        self.last_sent_linear = 0.0
        self.last_sent_angular = 0.0

        # 4. OCCUPANCY GRID MAP PARAMETERS (0.05m cell resolution)
        self.map_resolution = 0.05     # 5cm per cell array pixel
        self.map_width_meters = 10.0   # 10m wide map area coverage
        self.map_height_meters = 10.0  # 10m high map area coverage

        self.map_width_cells = int(self.map_width_meters / self.map_resolution)
        self.map_height_cells = int(self.map_height_meters / self.map_resolution)

        # Initialize the map message infrastructure
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "odom"
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = self.map_width_cells
        self.map_msg.info.height = self.map_height_cells
        
        # Center the grid origin perfectly around the startup point (0,0)
        self.map_msg.info.origin.position.x = -(self.map_width_meters / 2.0)
        self.map_msg.info.origin.position.y = -(self.map_height_meters / 2.0)
        self.map_msg.info.origin.position.z = 0.0
        
        # -1 initialization creates standard unknown empty gray grid blocks
        self.map_msg.data = [-1] * (self.map_width_cells * self.map_height_cells)

        # 5. SOCKET CONNECTIONS & ROS INTERFACES
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.LAPTOP_PORT))
        self.sock.setblocking(False)

        # Advertisers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriptions
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.manual_cmd_vel_callback, 10)

        # System Timers
        self.network_timer = self.create_timer(0.01, self.network_io_loop) # 100Hz processing speed
        self.map_publish_timer = self.create_timer(0.5, self.publish_map_callback) # 2Hz map refresh output
        
        self.get_logger().info("Integrated Low-Latency Map Controller with Goal-Marker Active.")

    def manual_cmd_vel_callback(self, msg: Twist):
        """ Intercepts standard manual teleop keyboard controls and overrides automated navigation """
        if self.navigation_active:
            self.get_logger().info("Manual control input received. Aborting current navigation path.")
            self.clear_goal_marker_from_map()
            self.navigation_active = False
        self.send_speeds_to_esp32(msg.linear.x, msg.angular.z)

    def goal_pose_callback(self, msg: PoseStamped):
        """ Receives target coordinates from 2D Goal Pose tool inside RViz interface """
        # Clear any old goal marker if another one was active before
        self.clear_goal_marker_from_map()

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.navigation_active = True
        self.get_logger().info(f"New target destination locked: X={self.goal_x:.2f}m, Y={self.goal_y:.2f}m")

        # Mark the new goal pose destination cell on the occupancy grid map
        self.mark_goal_on_grid(self.goal_x, self.goal_y)

    def network_io_loop(self):
        """ Low-overhead, high-frequency execution routine for reading incoming packets """
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                if len(data) == 8:
                    left_ticks, right_ticks = struct.unpack('<ii', data)
                    self.process_incoming_telemetry(left_ticks, right_ticks)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().error(f"Network interface processing error: {e}")
                break

        # Execute navigation logic step if an active target exists
        if self.navigation_active:
            self.execute_navigation_control_law()

    def process_incoming_telemetry(self, left_ticks, right_ticks):
        """ Translates raw hardware encoder ticks into odometry matrices and handles map tracing """
        if self.first_packet:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.first_packet = False
            return

        # Distance Tracking Calculations
        d_left = left_ticks - self.prev_left_ticks
        d_right = right_ticks - self.prev_right_ticks

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        dist_m1 = float(d_left) / self.TICKS_PER_METER
        dist_m2 = float(d_right) / self.TICKS_PER_METER

        # Direction signs adjustment matching the current control vector state
        if self.last_sent_linear < 0:
            dist_m1 = -dist_m1
            dist_m2 = -dist_m2
        elif self.last_sent_linear == 0.0 and self.last_sent_angular != 0.0:
            if self.last_sent_angular > 0:
                dist_m1 = -dist_m1
            else:
                dist_m2 = -dist_m2

        delta_s = (dist_m1 + dist_m2) / 2.0
        delta_th = (dist_m2 - dist_m1) / self.WHEEL_SEPARATION

        # Run Euler angular additions
        self.robot_x += delta_s * math.cos(self.robot_th + delta_th / 2.0)
        self.robot_y += delta_s * math.sin(self.robot_th + delta_th / 2.0)
        self.robot_th += delta_th

        # Normalize heading to stay within -PI to +PI bounds
        self.robot_th = math.atan2(math.sin(self.robot_th), math.cos(self.robot_th))

        # Convert state matrix outputs to system timestamps
        current_time = self.get_clock().now().to_msg()
        qz = math.sin(self.robot_th / 2.0)
        qw = math.cos(self.robot_th / 2.0)

        # 1. Update Global Transform Tree (/tf) - Immediate push removes RViz latency
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Standard Odometry Msg (/odom)
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # 3. Dynamic Tracking Mark on Map Grid Array (Robot path is black = 100)
        self.mark_pose_on_grid(self.robot_x, self.robot_y)

    def mark_pose_on_grid(self, real_x, real_y):
        """ Converts continuous spatial metrics into discrete map indices, marking trace cells as black (100) """
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y

        cell_x = int((real_x - origin_x) / self.map_resolution)
        cell_y = int((real_y - origin_y) / self.map_resolution)

        if (0 <= cell_x < self.map_width_cells) and (0 <= cell_y < self.map_height_cells):
            index = (cell_y * self.map_width_cells) + cell_x
            
            # If the robot rolls directly over the goal cell, clear the goal reference marker 
            # and let it turn into free space (0) or path (100)
            if index == self.current_goal_cell_index:
                self.map_msg.data[index] = 0
                self.current_goal_cell_index = None
                
            self.map_msg.data[index] = 100 # 100 = Black Occupied footprint path

    def mark_goal_on_grid(self, goal_x, goal_y):
        """ Encodes the current goal coordinates into a distinct intermediate value (60) to render green """
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y

        cell_x = int((goal_x - origin_x) / self.map_resolution)
        cell_y = int((goal_y - origin_y) / self.map_resolution)

        if (0 <= cell_x < self.map_width_cells) and (0 <= cell_y < self.map_height_cells):
            self.current_goal_cell_index = (cell_y * self.map_width_cells) + cell_x
            
            # Only overwrite if it hasn't already been traced by the robot footprint path
            if self.map_msg.data[self.current_goal_cell_index] != 100:
                self.map_msg.data[self.current_goal_cell_index] = 60  # 60 maps to bright costmap colors (Green/Yellow)

    def clear_goal_marker_from_map(self):
        """ Safets resets any old destination marker cell index back to standard free space """
        if self.current_goal_cell_index is not None:
            # If it wasn't stepped on by the robot, clean it up or mark it as free space (0)
            if self.map_msg.data[self.current_goal_cell_index] == 60:
                self.map_msg.data[self.current_goal_cell_index] = -1
            self.current_goal_cell_index = None

    def execute_navigation_control_law(self):
        """ Implements closed-loop proportional control tracking toward the goal coordinates """
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        
        # Euclidean distance error calculation
        distance_error = math.sqrt(dx*dx + dy*dy)

        # Destination check condition
        if distance_error < self.goal_tolerance:
            self.get_logger().info("Target destination reached successfully! Clearing goal marker.")
            self.send_speeds_to_esp32(0.0, 0.0)
            self.clear_goal_marker_from_map()
            self.navigation_active = False
            return

        # Target Heading calculation
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.robot_th
        
        # Wrap heading error to [-PI, PI] to prevent infinite spin adjustments
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # Steering priority check: If heading is off by > 45 deg, rotate in place first
        if abs(heading_error) > 0.8:
            linear_vel = 0.0
            angular_vel = self.Kp_angular * heading_error
        else:
            # Combined drive-and-steer closed loop law
            linear_vel = self.Kp_linear * distance_error
            angular_vel = self.Kp_angular * heading_error

        # Bound calculations against hardware constraint safety ceilings
        linear_vel = max(min(linear_vel, self.max_linear_speed), -self.max_linear_speed)
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)

        # Forward output straight to the hardware driver interface
        self.send_speeds_to_esp32(linear_vel, angular_vel)

    def send_speeds_to_esp32(self, linear_x, angular_z):
        """ Packs command arrays tightly into binary bytes and fires them out over UDP """
        self.last_sent_linear = linear_x
        self.last_sent_angular = angular_z

        packet = struct.pack('<ff', linear_x, angular_z)
        try:
            self.sock.sendto(packet, (self.ESP32_IP, self.ESP32_PORT))
        except Exception as e:
            self.get_logger().warn(f"Failed to transmit control payload: {e}")

    def publish_map_callback(self):
        """ Periodic low-frequency grid update publisher broadcast """
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    
    node = IntegratedRobotController()
    
    # Run the ROS 2 spin loop in a background thread so we can capture keys here
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    msg = """
========================================
   Navigate to Pose & Keyboard Teleop   
========================================

Control Your Robot Manually:
---------------------------
   w
a  s  d
   x

w/x : Move Forward / Backward
a/d : Turn Left / Right
s   : Force Stop

Speed Adjustments:
---------------------------
q/z : Increase/Decrease Linear Speed by 10%
e/c : Increase/Decrease Angular Speed by 10%

Note: Manual input will instantly cancel
any active autonomous navigation path.

CTRL-C to quit
========================================
"""
    print(msg)

    manual_linear_speed = node.max_linear_speed
    manual_angular_speed = node.max_angular_speed

    try:
        while True:
            key = getKey(settings)
            if key:
                if key == '\x03': # CTRL-C
                    break
                
                # If a key is pressed, override autonomous navigation
                if node.navigation_active:
                    print("\n[Manual Override] Canceling autonomous navigation.")
                    node.clear_goal_marker_from_map()
                    node.navigation_active = False
                
                twist = Twist()
                if key == 'w':
                    twist.linear.x = float(manual_linear_speed)
                elif key == 'x':
                    twist.linear.x = -float(manual_linear_speed)
                elif key == 'a':
                    twist.angular.z = float(manual_angular_speed)
                elif key == 'd':
                    twist.angular.z = -float(manual_angular_speed)
                elif key == 's':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == 'q':
                    manual_linear_speed = round(manual_linear_speed * 1.1, 4)
                    print(f"Linear speed increased to: {manual_linear_speed} m/s")
                    if node.last_sent_linear != 0.0:
                        twist.linear.x = float(manual_linear_speed) if node.last_sent_linear > 0 else -float(manual_linear_speed)
                    else:
                        continue
                elif key == 'z':
                    manual_linear_speed = round(manual_linear_speed * 0.9, 4)
                    print(f"Linear speed decreased to: {manual_linear_speed} m/s")
                    if node.last_sent_linear != 0.0:
                        twist.linear.x = float(manual_linear_speed) if node.last_sent_linear > 0 else -float(manual_linear_speed)
                    else:
                        continue
                elif key == 'e':
                    manual_angular_speed = round(manual_angular_speed * 1.1, 4)
                    print(f"Angular speed increased to: {manual_angular_speed} rad/s")
                    if node.last_sent_angular != 0.0:
                        twist.angular.z = float(manual_angular_speed) if node.last_sent_angular > 0 else -float(manual_angular_speed)
                    else:
                        continue
                elif key == 'c':
                    manual_angular_speed = round(manual_angular_speed * 0.9, 4)
                    print(f"Angular speed decreased to: {manual_angular_speed} rad/s")
                    if node.last_sent_angular != 0.0:
                        twist.angular.z = float(manual_angular_speed) if node.last_sent_angular > 0 else -float(manual_angular_speed)
                    else:
                        continue
                else:
                    # Ignore other keys
                    continue
                
                # Publish to ROS 2 network for echoing/debugging
                node.cmd_pub.publish(twist)
                # Send directly to hardware
                node.send_speeds_to_esp32(twist.linear.x, twist.angular.z)
                    
    except Exception as e:
        print(f"Teleop error: {e}")
    finally:
        print("\nShutting down... Stopping robot.")
        node.send_speeds_to_esp32(0.0, 0.0) # Full stop command safety catch on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()