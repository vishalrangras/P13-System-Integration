#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import math, tf

import copy
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.car_x = None
        self.car_y = None
        self.car_z = None

        self.orientation = None

        self.car_roll = None
        self.car_pitch = None
        self.car_yaw = None

        #self.frame_id = None
        #self.msg_seq = 1

        self.waypoints = None
        self.final_waypoints = Lane()
        self.no_waypoints = None

        self.closest = None #May remove it later
        
        rospy.spin()

    def get_closest_waypoint_index(self):
        if self.car_x is None or self.car_y is None or self.car_z is None or self.waypoints is None:
            return None
        distance = 1E10
        closest_waypoint_index = -1000
        for index, waypoint in enumerate(self.waypoints):
            waypoint_distance = math.sqrt((self.car_x - wp.pose.pose.position.x)**2 +\
                                          (self.car_y - wp.pose.pose.position.y)**2 +\
                                          (self.car_z - wp.pose.pose.position.z)**2)

            if waypoint_distance < distance:
                distance = waypoint_distance
                closest_waypoint_index = index
        return index

    def get_local_coordinates(self, waypoint):
        x, y = waypoint.pose.pose.position.x - self.car_x, waypoint.pose.pose.position.y - self.car_y
        x_local = math.cos(self.car_yaw) * x + math.sin(self.car_yaw) * y
        y_local = -math.sin(self.car_yaw) * x + math.cos(self.car_yaw) * y
        return x_local, y_local

    def get_next_waypoints(self):
        closest_waypoint_index = self.get_closest_waypoint_index()

        if self.car_yaw is None or self.waypoints is None or closest_waypoint_index is None:
            return None

        waypoint = self.waypoints[closest_waypoint_index]
        x_local, y_local = self.get_local_coordinates(waypoint)

        if x_local < 0:
            closest_waypoint_index = (closest_waypoint_index + 1) % self.no_waypoints

        lane = Lane()
        for i in range(LOOKAHEAD_WPS):
            lane.waypoints.append(self.waypoints[(closest_waypoint_index + i) % self.no_waypoints])

        return lane


    def pose_cb(self, msg):
        # TODO: Implement
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y
        self.car_z = msg.pose.position.z

        self.orientation = msg.pose.orientation
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        self.car_roll, self.car_pitch, self.car_yaw = tf.transformations.euler_from_quaternion(quaternion)

        if self.final_waypoints is not None:
            self.final_waypoints_pub.publish(self.final_waypoints)
        else:
            self.final_waypoints = self.get_next_waypoints()
            self.final_waypoints_pub.publish(self.final_waypoints)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = msg.waypoints
        self.no_waypoints = len(mgs.waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist




if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
