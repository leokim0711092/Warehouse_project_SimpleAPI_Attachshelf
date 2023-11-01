import time
from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from custom_interfaces.srv import GoToLoading
from geometry_msgs.msg import Polygon, Point32
from rclpy.node import Node


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult



class ServiceAndPublisherNode(Node):

    def __init__(self):
        super().__init__('service_and_publisher_node')

        # Create a publisher for the custom footprint
        self.publisher_local = self.create_publisher(Polygon, 'local_costmap/footprint', 10)
        self.publisher_global = self.create_publisher(Polygon, 'global_costmap/footprint', 10)


    def publish_custom_footprint(self, length):
        new_footprint = self.create_custom_footprint(length)

        # Publish the updated footprint
        self.publisher_local.publish(new_footprint)
        self.publisher_global.publish(new_footprint)
        self.get_logger().info('publish_footprint')

    def create_custom_footprint(self, length):
            
        new_footprint = Polygon()
        # new_footprint.header.frame_id = 'robot_odom'  # Specify the frame_id 

        point1 = Point32()
        point1.x = length / 2.0
        point1.y = length / 2.0
        point1.z = 0.0

        point2 = Point32()
        point2.x = -length / 2.0
        point2.y = length / 2.0
        point2.z = 0.0

        point3 = Point32()
        point3.x = -length / 2.0
        point3.y = -length / 2.0
        point3.z = 0.0

        point4 = Point32()
        point4.x = length / 2.0
        point4.y = -length / 2.0
        point4.z = 0.0

        # Add the points to the polygon
        new_footprint.points.append(point1)
        new_footprint.points.append(point2)
        new_footprint.points.append(point3)
        new_footprint.points.append(point4)

        return new_footprint


def main():


    rclpy.init()

    node = ServiceAndPublisherNode()

    node.publish_custom_footprint(0.7)
        
    node.destroy_node()


if __name__ == '__main__':
    main()