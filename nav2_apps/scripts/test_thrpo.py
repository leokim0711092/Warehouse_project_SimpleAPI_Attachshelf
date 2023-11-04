import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

'''
Basic security route patrol demo. In this demonstration, the expectation
is that there are security cameras mounted on the robots recording or being
watched live by security staff.
'''


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Security route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    security_route = [
        [4.54, 1.4, 1.0 , 0.0],
        [2.28, 0.218, 1.0, 0.0],
        [1.145, -0.62, 0.92 , 0.38],
        [0.8, -2.2, 0.71, 0.71]
        ]

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 5.84
    initial_pose.pose.position.y = 0.07
    initial_pose.pose.orientation.z = -0.706
    initial_pose.pose.orientation.w = 0.707
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully 
    navigator.waitUntilNav2Active()

    # Do security route until dead

    # Send your route
    route_poses = []
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    for pt in security_route:
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation.z = pt[2]
        pose.pose.orientation.w = pt[3] 

        route_poses.append(deepcopy(pose))
    navigator.goThroughPoses(route_poses)

    # Do something during your route (e.x. AI detection on camera images for anomalies)
    # Print ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

            # Some failure mode, must stop since the robot is clearly stuck
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                print('Navigation has exceeded timeout of 180s, canceling the request.')
                navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Route complete! Restarting...')
    elif result == TaskResult.CANCELED:
        print('Security route was canceled, exiting.')
        exit(1)
    elif result == TaskResult.FAILED:
        print('Security route failed! Restarting from the other side...')

    exit(0)


if __name__ == '__main__':
    main()