#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import RobotState, RobotMode, Location
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import math

class RMFRobotStatePublisher(Node):
    def __init__(self):
        super().__init__('rmf_robot_state_publisher')
        
        # Robot configuration
        self.robot_name = "minibot_1"
        self.robot_model = "minibot"
        self.level_name = "L1"
        
        # State variables
        self.seq = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.battery_percent = 100.0
        self.robot_mode = RobotMode.MODE_IDLE
        
        # Publishers
        self.robot_state_pub = self.create_publisher(
            RobotState, 
            '/robot_state', 
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/base_controller/odom',
            self.odom_callback,
            10
        )
        
        # Optional battery subscriber (if available)
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        
        # Timer to publish robot state at 10Hz
        self.timer = self.create_timer(0.1, self.publish_robot_state)
        
        self.get_logger().info(f'RMF Robot State Publisher started for {self.robot_name}')
    
    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        self.current_yaw = yaw
        
        # Update mode based on velocity
        linear_vel = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        angular_vel = abs(msg.twist.twist.angular.z)
        
        if linear_vel > 0.01 or angular_vel > 0.01:
            self.robot_mode = RobotMode.MODE_MOVING
        else:
            self.robot_mode = RobotMode.MODE_IDLE
    
    def battery_callback(self, msg):
        """Update battery state"""
        self.battery_percent = msg.percentage * 100.0
    
    def publish_robot_state(self):
        """Publish RMF RobotState message"""
        robot_state = RobotState()
        
        # Basic info
        robot_state.name = self.robot_name
        robot_state.model = self.robot_model
        robot_state.task_id = ""
        robot_state.seq = self.seq
        self.seq += 1
        
        # Mode
        robot_state.mode.mode = self.robot_mode
        robot_state.mode.mode_request_id = 0
        
        # Battery
        robot_state.battery_percent = self.battery_percent
        
        # Location
        location = Location()
        location.t = self.get_clock().now().to_msg()
        location.x = self.current_x
        location.y = self.current_y
        location.yaw = self.current_yaw
        location.level_name = self.level_name
        location.index = 0
        
        robot_state.location = location
        robot_state.path = []  # Empty path for now
        
        # Publish
        self.robot_state_pub.publish(robot_state)

def main(args=None):
    rclpy.init(args=args)
    node = RMFRobotStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
