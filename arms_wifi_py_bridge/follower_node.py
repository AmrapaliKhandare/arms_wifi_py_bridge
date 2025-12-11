#!/usr/bin/env python3
# Copyright 2025 Trossen Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import trossen_arm
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class FollowerArmNode(Node):
    def __init__(self):
        super().__init__('follower_arm_node')
        
        # Declare parameters
        self.declare_parameter('server_ip', '192.168.1.3')
        self.declare_parameter('publish_rate', 100.0)  # Hz
        
        # Get parameters
        self.server_ip = self.get_parameter('server_ip').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.get_logger().info(f'Follower arm server IP: {self.server_ip}')
        
        # Initialize driver
        self.get_logger().info('Initializing follower arm driver...')
        self.driver = trossen_arm.TrossenArmDriver()
        self.driver.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_follower,
            self.server_ip,
            False
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # UDP-like (drops packets if needed)
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # Only keep the SINGLE latest message
        )
        
        # Publishers and Subscribers
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/follower/external_efforts',
            qos_profile
        )
        self.position_sub = self.create_subscription(
            Float64MultiArray,
            '/leader/joint_positions',
            self.position_callback,
            qos_profile
        )
        self.velocity_sub = self.create_subscription(
            Float64MultiArray,
            '/leader/joint_velocities',
            self.velocity_callback,
            qos_profile
        )
        
        # State variables
        self.teleop_active = False
        self.latest_velocities = None
        self.shutdown_requested = False  # Flag to prevent restart after emergency
        
        self.last_packet_time = self.get_clock().now()
        # Move to home position
        self.move_to_home()
        
        # Create timer for publishing follower efforts
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.get_logger().info('Follower arm node initialized and ready!')
    
    def emergency_stop(self):
        """Emergency stop - immediately go to sleep position"""
        self.get_logger().error('EMERGENCY STOP - Moving to sleep position')
        self.teleop_active = False
        self.shutdown_requested = True  # Prevent any restart
        try:
            self.driver.set_all_modes(trossen_arm.Mode.position)
            self.driver.set_all_positions(
                np.zeros(self.driver.get_num_joints()),
                5.0,
                True
            )
            self.get_logger().info('Follower at sleep position')
        except Exception as e:
            self.get_logger().error(f'Failed to move to sleep: {e}')
    
    def move_to_home(self):
        """Move the follower arm to home position"""
        self.get_logger().info('Moving follower to home position...')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(
            np.array([0.0, np.pi/2, np.pi/2, 0.0, 0.0, 0.0, 0.0]),
            5.0,
            True
        )
        self.get_logger().info('Follower at home position')
    
    def move_to_sleep(self):
        """Move the follower arm to sleep position"""
        self.get_logger().info('Moving follower to sleep position...')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(
            np.zeros(self.driver.get_num_joints()),
            5.0,
            True
        )
        self.get_logger().info('Follower at sleep position')
    
    def start_teleoperation(self):
        """Start teleoperation mode"""
        if not self.teleop_active:
            self.get_logger().info('Starting teleoperation mode...')
            self.driver.set_all_modes(trossen_arm.Mode.position)
            self.teleop_active = True
    
    def stop_teleoperation(self):
        """Stop teleoperation mode"""
        if self.teleop_active:
            self.get_logger().info('Stopping teleoperation mode...')
            self.teleop_active = False
            self.shutdown_requested = True  # Prevent restart
            self.move_to_home()
            self.move_to_sleep()
    
    def timer_callback(self):

        if self.shutdown_requested:
            return

        # NEW: WATCHDOG CHECK
        # If we haven't heard from the leader in 1.0 second, shut down
        # This will trigger when the Leader enters the "Delay Phase"
        if self.teleop_active:
            silence_duration = (self.get_clock().now() - self.last_packet_time).nanoseconds / 1e9
            if silence_duration > 1.0:
                self.get_logger().warn('No data from Leader for 1.0s. Initiating safety shutdown.')
                self.stop_teleoperation()
                return
            
        """Publish follower external efforts at fixed rate"""
        # Check if shutdown was requested (emergency stop)
        if self.shutdown_requested:
            return  # Stop all operations permanently
        
        # Always publish efforts (even when not teleoperating)
        # This allows leader to detect that follower is alive
        try:
            efforts = self.driver.get_all_external_efforts()
            
            # Convert VectorDouble to list
            efforts_list = [efforts[i] for i in range(len(efforts))]
            
            effort_msg = Float64MultiArray()
            effort_msg.data = efforts_list
            self.effort_pub.publish(effort_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to read/publish efforts: {e}', throttle_duration_sec=1.0)
            self.emergency_stop()
    
    def position_callback(self, msg):

        self.last_packet_time = self.get_clock().now()

        """Receive positions from leader and apply to follower"""
        # Check if shutdown was requested
        if self.shutdown_requested:
            return  # Ignore all commands after shutdown
        
        if not self.teleop_active:
            self.start_teleoperation()
        
        positions = np.array(msg.data)
        
        # SAFETY CHECK: Validate positions are within reasonable bounds
        # Joint limits for WXAI V0: approximately [-2π, 2π] for most joints
        max_angle = 6.28  # ~2π radians
        if np.any(np.abs(positions) > max_angle):
            self.get_logger().error(f'Received invalid position command: {positions}')
            self.get_logger().error('Position exceeds joint limits! Emergency stop.')
            self.emergency_stop()
            return
        
        try:
            # Apply positions with velocities if available
            if self.latest_velocities is not None:
                self.driver.set_all_positions(
                    positions,
                    0.0,
                    False,
                    self.latest_velocities
                )
            else:
                self.driver.set_all_positions(
                    positions,
                    0.0,
                    False
                )
        except Exception as e:
            self.get_logger().error(f'Failed to set positions: {e}')
            self.emergency_stop()
    
    def velocity_callback(self, msg):
        """Store latest velocities from leader"""
        self.latest_velocities = np.array(msg.data)
    
    def destroy_node(self):
        """Cleanup before shutting down"""
        self.get_logger().info('Shutting down follower arm node...')
        if self.teleop_active:
            self.emergency_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowerArmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:

        if node.shutdown_requested:
            node.get_logger().info('Clean exit (already shut down).')
        else:
            node.get_logger().info('Ctrl+C detected! Parking follower arm safely...')
            
            # 1. Stop Logic
            node.teleop_active = False 
            node.shutdown_requested = True
            
            # 2. Park Immediately (Home -> Sleep)
            try:
                node.move_to_home()
                node.move_to_sleep()
            except Exception as e:
                node.get_logger().error(f'Error during park: {e}')
            
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()