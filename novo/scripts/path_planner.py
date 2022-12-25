#! /usr/bin/env python3

# Python Imports

# ROS Imports
from novo_interfaces.msg import Path
from novo_interfaces.srv import GetPath

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node

# Personal Imports

##############################################################################

class PathPlannerService(Node):
    def __init__(self) -> None:
        super().__init__('path_planner_service')

        self.srv = self.create_service(GetPath, 'path_planner', self.path_planner_callback)
        return

    ###### Callbacks ######

    def path_planner_callback(self, request, response: GetPath.Response) -> GetPath.Response:
        # TODO: implement the path planner
        response.path = Path(
            path=[Point(x=0.0, y=0.0, z=0.0)])
        self.get_logger().info('Incoming request for path')
        return response

    ###### Methods ######

##############################################################################

def main() -> None:
    print("Initializing Path Planner...")
    rclpy.init()

    path_planner = PathPlannerService()
    
    print("Spinning Path Planner...")
    rclpy.spin(path_planner)

    print("Destroying Path Planner...")
    path_planner.destroy_node()

    rclpy.shutdown()
    return

if __name__ == "__main__":
    main()