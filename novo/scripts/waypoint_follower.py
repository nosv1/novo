#! /usr/bin/env python3

# Python Imports
from math import pi
from random import randint

# ROS Imports
from novo_interfaces.msg import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

# Personal Imports
from novo_support.math_tools import clamp
from novo_support.PID import PID
from novo_support.quaternion_tools import euler_from_quaternion
from novo_support.TurtleNode import Turtle
from novo_support.Waypoint import Waypoint

##############################################################################

class WaypointFollower(Node):
    def __init__(self, *,
        turtle: Turtle,
        heading_PID: PID, 
        max_speed: float, 
        max_turn_rate: float) -> None:
        """
        Initializes the waypoint follower
        :param turtle: The turtle to control
        :param heading_PID: The PID controller for the heading
        :param max_speed: The maximum speed the turtle can move
        :param max_turn_rate: The maximum turn rate the turtle can turn
        """
        super().__init__('waypoint_follower')
        
        self.turtle = turtle
        self.heading_PID = heading_PID
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate

        ###### Odom Subscriber ######

        self.odom_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic=f'{turtle.namespace}/odom',
            callback=self.odom_callback,
            qos_profile=10)
        self.odom_timestamp: float = 0.0
        self.odom_dt: float = 0.0
        self.position: Point = Point()
        self.orientation: Quaternion = Quaternion()
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0
        print(f"{self.get_name()} subscribed to {self.odom_subscriber.topic_name}")

        ###### Path Subscriber ######

        self.path_subscriber = self.create_subscription(
            msg_type=Path,
            topic=f'{turtle.namespace}/path',
            callback=self.path_callback,
            qos_profile=10)
        self.path_timestamp: float = 0.0
        self.path_dt: float = 0.0
        self.path: list[Point] = list()
        self.current_waypoint: Waypoint = None
        return

    ###### Callbacks ######

    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback for the odom subscriber
        :param msg: The odom message
        """
        self.odom_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_dt = self.get_clock().now().nanoseconds * 1e-9 - self.odom_timestamp

        self.orientation = msg.pose.pose.orientation
        self.position = msg.pose.pose.position
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            x=self.orientation.x,
            y=self.orientation.y,
            z=self.orientation.z,
            w=self.orientation.w)

        if not self.current_waypoint is None:
            self.follow_waypoint()
        return

    def path_callback(self, msg: Path) -> None:
        """
        Callback for the path subscriber
        :param msg: The path message
        """
        path: list[Point] = msg.path
        self.current_waypoint = Waypoint(path[0])
        return

    ###### Methods ######

    def follow_waypoint(self) -> None:
        """
        Follows the current waypoint
        """
        # get the desired heading
        desired_heading: float = Waypoint(self.position).heading_to(self.current_waypoint.point)

        # decide to turn left or right
        desired_heading = (
            desired_heading - 2 * pi
            if desired_heading - self.yaw > pi
            else desired_heading
        )

        self.turtle.twist.angular.z = clamp(
            self.heading_PID.update(
                desired=desired_heading, actual=self.yaw, dt=self.odom_dt
            ), -self.max_turn_rate, self.max_turn_rate)

        self.turtle.twist.linear.x = self.max_speed

        self.turtle.move()
        return

##############################################################################

def main() -> None:
    print("Initializing Waypoint Follower...")
    rclpy.init()

    waypoint_follower: WaypointFollower = WaypointFollower(
        turtle=Turtle(namespace='', name="Turtle"),
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        max_speed=0.22,  # max irl turtle speed
        max_turn_rate=1.4)  # matches constriant for 0.3 m radius turning at max speed

    # waypoint_follower.current_waypoint = Waypoint(Point(x=0.0, y=0.0))

    try:
        print("Spinning Waypoint Follower...")
        while rclpy.ok():
            rclpy.spin_once(waypoint_follower)

            # new random waypoint when current waypoint is reached
            # if waypoint_follower.current_waypoint.contains(waypoint_follower.position):
            #     waypoint_follower.current_waypoint = Waypoint(Point(
            #         x=float(randint(-5.0, 5.0)),
            #         y=float(randint(-5.0, 5.0))),
            #         radius=0.25)
            #     print(f"New Waypoint: {waypoint_follower.current_waypoint.point}")
    except KeyboardInterrupt:
        pass

    print("Destroying Waypoint Follower Turtle...")
    waypoint_follower.turtle.destroy_node()

    print(f"Destroying Waypoint Follower...")
    waypoint_follower.destroy_node()

    rclpy.shutdown()
    return

if __name__ == "__main__":
    main()