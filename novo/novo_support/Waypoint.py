#! /usr/bin/env python3

# Python Imports
from math import atan2

# ROS Imports
from geometry_msgs.msg import Point

class Waypoint:
    def __init__(self, 
        point: Point, 
        exit_heading: float = None,
        radius: float = 1) -> None:
        """
        Initializes the waypoint
        :param point: The point of the waypoint
        :param exit_heading: The heading the turtle should have when it exits the waypoint
        :param radius: The radius of the waypoint in meters (default: 1)
        """
        self.point = point
        self.exit_heading = exit_heading
        self.radius = radius
        return

    def heading_to(self, point: Point) -> float:
        """
        Returns the heading to the given point
        :param point: The point to get the heading to
        :return: The heading to the given point
        """
        return atan2(point.y - self.point.y, point.x - self.point.x)

    def distance_to(self, point: Point) -> float:
        """
        Returns the distance to the given point
        :param point: The point to get the distance to
        :return: The distance to the given point
        """
        return ((point.x - self.point.x)**2 + (point.y - self.point.y)**2)**0.5

    def contains(self, point: Point) -> bool:
        """
        Returns whether the given point is within the waypoint
        :param point: The point to check
        :return: Whether the given point is within the waypoint radius
        """
        return self.distance_to(point) <= self.radius