#!/usr/bin/env python3
"""
Goal Relay Node - Fixes timestamp issues with Nav2 goal poses.

This node subscribes to goal poses from RViz and republishes them with 
the current timestamp (Time(0)), which tells Nav2 to use the latest 
available transform instead of trying to look up historical transforms.

This fixes the "Extrapolation Error looking up target frame" issue that
occurs when the TF buffer doesn't have old enough data.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup


class GoalRelayNode(Node):
    def __init__(self):
        super().__init__('goal_relay_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribe to RViz goal pose topic
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action client to send goals to Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Goal Relay Node started - will fix goal timestamps')
        self.get_logger().info('Use RViz "2D Goal Pose" to send navigation goals')
    
    def goal_callback(self, msg: PoseStamped):
        """Receive goal from RViz and send to Nav2 with current timestamp."""
        
        self.get_logger().info(
            f'Received goal at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            return
        
        # Create goal with CURRENT timestamp (Time 0 = use latest transform)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = msg.header.frame_id
        goal_msg.pose.header.stamp = Time(seconds=0).to_msg()  # Use latest transform!
        goal_msg.pose.pose = msg.pose
        
        self.get_logger().info(
            f'Sending goal with timestamp=0 (use latest transform)'
        )
        
        # Send goal asynchronously
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        
        self.get_logger().info('Goal accepted by Nav2')
        
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose.position
        distance = feedback.distance_remaining
        self.get_logger().info(
            f'Position: ({current_pose.x:.2f}, {current_pose.y:.2f}), '
            f'Distance remaining: {distance:.2f}m',
            throttle_duration_sec=2.0
        )
    
    def result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('🎯 Navigation succeeded!')
        elif result.status == 5:  # CANCELED
            self.get_logger().info('Navigation canceled')
        elif result.status == 6:  # ABORTED
            self.get_logger().warn('Navigation aborted')
        else:
            self.get_logger().info(f'Navigation finished with status: {result.status}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
