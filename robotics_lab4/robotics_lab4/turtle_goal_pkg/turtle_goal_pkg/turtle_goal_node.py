#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose


class TurtleGoalNode(Node):

    def __init__(self):
        super().__init__('turtle_goal_node')

        self.pose = None
        self.goal = None

        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)

        self.goal_sub = self.create_subscription(
            Point,
            '/turtle1/goal',
            self.goal_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Turtle Goal Node started!')

    def pose_callback(self, msg):
        self.pose = msg

    def goal_callback(self, msg):
        self.goal = msg
        self.get_logger().info(f'New goal received: x={msg.x:.2f}, y={msg.y:.2f}')

    def control_loop(self):
        if self.pose is None or self.goal is None:
            return

        dx = self.goal.x - self.pose.x
        dy = self.goal.y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Goal reached!')
            return

        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        Kp_linear = 1.5
        Kp_angular = 6.0

        cmd = Twist()
        cmd.angular.z = Kp_angular * angle_error

        if abs(angle_error) < 0.3:
            cmd.linear.x = Kp_linear * distance

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
