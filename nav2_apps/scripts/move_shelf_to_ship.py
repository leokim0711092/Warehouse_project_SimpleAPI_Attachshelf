import time
from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

#Initial position
Initial_position = {
    "position": [0.073, -0.05],
    "orientation": [0.0, 0.0, 0.0, 1.0]
    }
desired_rotation_angle = math.pi / 2.0

# Shelf positions for loading
loading_position = {
    "position": [5.62, 0.94],
    "orientation": [ 0.0, 0.0 , math.sin(desired_rotation_angle/2 ),math.cos(desired_rotation_angle/2 )]
    }

# Shipping destination for picked products
shipping_destinations = {
    "recycling": [-0.205, 7.403],
    "pallet_jack7": [-0.073, -8.497],
    "conveyer_432": [6.217, 2.153],
    "frieght_bay_3": [-6.349, 9.147]}



def main():

    request_destination = 'shipping_position'
    ####################

    rclpy.init()

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
        print('Arrive at loading position! Bringing product to shipping destination (' + request_destination + ')...')

        # shipping_destination = PoseStamped()
        # shipping_destination.header.frame_id = 'map'
        # shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        # shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
        # shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
        # shipping_destination.pose.orientation.z = 1.0
        # shipping_destination.pose.orientation.w = 0.0
        # navigator.goToPose(shipping_destination)

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


if __name__ == '__main__':
    main()