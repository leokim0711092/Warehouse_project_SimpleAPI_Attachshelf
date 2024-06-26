import time
from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.duration import Duration
import rclpy
from custom_interfaces.srv import GoToLoading
from geometry_msgs.msg import Polygon, Point32
from rclpy.node import Node


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

#Initial position
Initial_position = {
    "position": [-3.0, -0.7],
    "orientation": [0.0, 0.0, 0.38, 0.92]
    }
desired_rotation_angle = math.pi / 2.0

# Shelf positions for loading
loading_position = {
    "position": [0.92148, 0.8507],
    "orientation": [ 0.0, 0.0 , -0.4883, 0.872673 ]
    }

# Secuirty route
security_route = {
    "position": [1.43043, 0.243886],
    "orientation": [ 0.0, 0.0 ,  0.926911, 0.375282 ]
    }

# security_route = [
#     [1.2585, 0.543, -0.747, 0.664]
#     ]

security_route_2 = [
    # [1.307, 0.178, 0.988, 0.1517]
    [0.226, 0.94, -0.929, 0.369],
    [-2.08, -0.89, -0.59 , 0.801],
    [-1.45, -3.15, 0.918, 0.396],
    ]

class ServiceAndPublisherNode(Node):

    def __init__(self):
        super().__init__('service_and_publisher_node')

        # Create a service client
        self.service_client = self.create_client(GoToLoading, 'approach_shelf')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service server is not available. Waiting...')

        # Create a publisher for the custom footprint
        self.publisher_local = self.create_publisher(Polygon, 'local_costmap/footprint', 10)
        self.publisher_global = self.create_publisher(Polygon, 'global_costmap/footprint', 10)
        self.publisher_ele_down = self.create_publisher(String, 'elevator_down', 10)

    def ele_down(self):
        ele_dn = String()
        self.publisher_ele_down.publish(ele_dn)

    def call_approach_shelf(self):

        request = GoToLoading.Request()
        request.attach_to_shelf = True
        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        length = None

        if future.result() is not None:
            response = future.result()
            if response.complete:
                self.get_logger().info('Service call succeeded')
                length = response.length
                self.publish_custom_footprint(length)
                print('length %f' % (length))
            else:
                self.get_logger().error('Service call failed')
        else:
            self.get_logger().error('Service call failed')
            
        return length        


    def publish_custom_footprint(self, length):
        new_footprint = self.create_custom_footprint(length)

        # Publish the updated footprint
        self.publisher_local.publish(new_footprint)
        self.publisher_global.publish(new_footprint)
        self.get_logger().info('publish_footprint')

    def create_custom_footprint(self, length):
            
        new_footprint = Polygon()
        # new_footprint.header.frame_id = 'robot_odom'  # Specify the frame_id 
        length = length*0.85
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

    def publish_circle_footprint(self, radius):
        new_footprint = self.create_circle_footprint(radius)

        # Publish the updated footprint
        self.publisher_local.publish(new_footprint)
        self.publisher_global.publish(new_footprint)
        self.get_logger().info('publish circle new footprint')

    def create_circle_footprint(self, radius):
        new_footprint = Polygon()

        # Number of sides to approximate the circle
        num_sides = 60  # You can adjust this to change the circle's smoothness

        for i in range(num_sides):
            angle = (2 * 3.14159265359 * i) / num_sides
            point = Point32()
            point.x = radius * math.cos(angle)
            point.y = radius * math.sin(angle)
            point.z = 0.0
            new_footprint.points.append(point)

        return new_footprint



def main():
    ####################

    rclpy.init()

    node = ServiceAndPublisherNode()
    navigator = BasicNavigator()

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = Initial_position['position'][0]
    initial_pose.pose.position.y = Initial_position['position'][1]
    initial_pose.pose.orientation.x = Initial_position['orientation'][0]
    initial_pose.pose.orientation.y = Initial_position['orientation'][1]
    initial_pose.pose.orientation.z = Initial_position['orientation'][2]
    initial_pose.pose.orientation.w = Initial_position['orientation'][3]
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = loading_position['position'][0]
    shelf_item_pose.pose.position.y = loading_position['position'][1]
    shelf_item_pose.pose.orientation.x = loading_position['orientation'][0]
    shelf_item_pose.pose.orientation.y = loading_position['orientation'][1]
    shelf_item_pose.pose.orientation.z = loading_position['orientation'][2]
    shelf_item_pose.pose.orientation.w = loading_position['orientation'][3]
    print('Received request for going to loading position')
    navigator.goToPose(shelf_item_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at loading position' +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    print('Arrive at loading position!')

    node.call_approach_shelf() 
    print('Loading shelf successfully')

    # Security outside

    security_pose = PoseStamped()
    security_pose.header.frame_id = 'map'
    security_pose.header.stamp = navigator.get_clock().now().to_msg()
    security_pose.pose.position.x = security_route['position'][0]
    security_pose.pose.position.y = security_route['position'][1]
    security_pose.pose.orientation.x = security_route['orientation'][0]
    security_pose.pose.orientation.y = security_route['orientation'][1]
    security_pose.pose.orientation.z = security_route['orientation'][2]
    security_pose.pose.orientation.w = security_route['orientation'][3]
    print('Received request for going to loading position')
    navigator.goToPose(security_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at loading position' +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    print('Arrive at security place!')
        
        # Send your route
    route_poses_1 = []
    pose_1 = PoseStamped()
    pose_1.header.frame_id = 'map'
    pose_1.header.stamp = navigator.get_clock().now().to_msg()

    for pt in security_route_2:
        pose_1.pose.position.x = pt[0]
        pose_1.pose.position.y = pt[1]
        pose_1.pose.orientation.z = pt[2]
        pose_1.pose.orientation.w = pt[3] 

        route_poses_1.append(deepcopy(pose_1))
    navigator.goThroughPoses(route_poses_1)
    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at loading position' +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.') 

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:

        node.ele_down()
        print('Put shelt down')

        node.publish_circle_footprint(0.15)

        navigator.goToPose(initial_pose)
        print('Return to Initial position')
        print('Task succeed')

    elif result == TaskResult.CANCELED:
        print('Task at loading position was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at loading position failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)
    node.destroy_node()


if __name__ == '__main__':
    main()