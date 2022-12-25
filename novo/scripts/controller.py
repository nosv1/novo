#! /usr/bin/env python3
"""
Controller controls the 'Novo' navigation system
The navigation system is composed of the following nodes:
    - Controller
    - Path Planner
    - Waypoint Follower
        - Turtle
    - Object Tracker
    - turtlebot3_navigation2

The Controller Process:
    1. Listen for new map images from the turtlebot3_navigation2 node.
    2. Request updated tracked objects from Object Tracker.
        - Send new and previous map images
        - Receive list of tracked objects (points with velocities)
    3. Request updated path from Path Planner.
        - Send current map image, current position, and current tracked objects
        - Receive list of waypoints
    4. Send updated path to Waypoint Follower.
"""

# Python Imports

# ROS Imports
from geometry_msgs.msg import Point

from novo_interfaces.msg import Path
from novo_interfaces.srv import GetPath
# from novo_interfaces.srv import ObjectTracker

import rclpy
from rclpy.task import Future
from rclpy.node import Node
from rclpy.publisher import Publisher

# Personal Imports

##############################################################################

class Controller(Node):
    """
    __init__ defines:
        request clients:
            - path_planner_client
        publishers:
            - path_publisher
        subscribers:
            - map_image_subscriber # TODO
    """
    def __init__(self) -> None:
        super().__init__('controller')

        self.path_planner_client = self.create_client(GetPath, 'path_planner')
        while not self.path_planner_client.wait_for_service(timeout_sec=1.0):
            (self
                .get_logger()
                .info('path planner service not available, waiting again...'))
        self.path_planner_request = GetPath.Request()
        self.path_publisher: Publisher = self.create_publisher(Path, 'path', 10)

        return

    ###### Requests ######

    def request_path(self) -> None:
        # TODO: set the parameters of the request
        # self.path_planner_request.map_image = None
        # self.path_planner_request.current_position = None
        # self.path_planner_request.tracked_objects = None
        self.path_planner_future: Future = (self.path_planner_client
            .call_async(self.path_planner_request))
        return
    
    ###### Callbacks ######

    def path_planner_callback(self, future: Future) -> None:
        try:
            response = future.result()
            self.publish_path(response.path)
        except Exception as e:
            (self
                .get_logger()
                .info('Path Planner service call failed %r' % (e,)))
        else:
            (self
                .get_logger()
                .info('Path Planner service call succeeded!'))
        return

    ###### Publishers ######

    def publish_path(self, path: Path) -> None:
        self.path_publisher.publish(path)
        return

    ###### Methods ######

##############################################################################

def main() -> None:
    print("Initializing Controller...")
    rclpy.init()

    controller = Controller()
    controller.request_path()  # TODO: in reality, we can't request a path until we at least get a map image

    print("Spinning Controller...")
    while rclpy.ok():
        rclpy.spin_once(controller, timeout_sec=1.0)
        if controller.path_planner_future.done():
            controller.path_planner_callback(controller.path_planner_future)
            controller.path_planner_future = None
            controller.request_path()

    print("Destroying Controller...")
    controller.destroy_node()
    rclpy.shutdown()
    return

if __name__ == "__main__":
    main()