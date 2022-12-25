#! /usr/bin/env python3

# python imports

# ros2 imports
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist

class Turtle(Node):
    def __init__(self, namespace='', name='Turtle') -> None:
        super().__init__(name)

        self.namespace = namespace
        self.name = name

        ###### Cmd Vel Publisher ######
        
        self.cmd_vel_publisher: Publisher = self.create_publisher(
            Twist, f"{namespace}/cmd_vel", 10)
        print(f"{self.get_name()} publishing to '{self.cmd_vel_publisher.topic_name}'")
        self.twist: Twist = Twist()

        return

    ###### Publishers ######
    def move(self) -> None:
        self.cmd_vel_publisher.publish(self.twist)
        return

    ###### Methods ######

    def set_time(self) -> None:
        self.previous_wall_time = self.current_wall_time
        self.current_wall_time = self.get_clock().now().nanoseconds / 1e9
        return