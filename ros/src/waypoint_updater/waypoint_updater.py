#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math
import numpy as np
from scipy.spatial import KDTree

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5 # calibration data

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        # reserve waypoint
        self.base_lane = None
        # get from traffic_waypoint
        self.stopline_wp_idx = -1
        # contains a list of (x,y) tuples for all waypoints
        self.waypoints_2d = None
        # KD tree of the x,y waypoints to increase lookup time
        self.waypoint_tree = None
        # stores the raw pose message        
        self.pose_msg = None
       
       
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # this is a new subscriber , traffic info
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)  

        # publish the final waypoints , out put the generate lane       
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # the interval time should at lease 200ms
        self.publisher_loop(50)

    def publisher_loop(self, frequency):
        """
        Task: This method is called from the constructor and is responsible for calling the
              publishers and their helpers repeatedly.
        arguments:
        -frequency: int, the frequency with which to call the publishers
        returns: Nothing
        
        """
        rate = rospy.Rate(frequency)

        while not rospy.is_shutdown():
            if self.pose_msg and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        """
        
        Task: Processes the messages which contain the current 
              position of the vehicle in map coordinates
        arguments:
        - msg: message type geometry_msgs/PoseStamped
        returns: Nothing
        ROS integration
        ===        
        Type: Callback
        Topic: /current_pose
        msg_type: geometry_msgs/PoseStamped
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Pose pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w        
        """
        self.pose_msg = msg
    
    def waypoints_cb(self, waypoint_msg):
        """
        Task: Processes the waypoints message which contains all of the track's waypoints in map coordinates. 
              Needs only to run once, because the waypoints are sent only once at the beginning.
        arguments:
        - waypoints: message type styx_msgs/Lane
        returns: Nothing
        
        ROS integration:
        ===
        Type: Callback
        Topic: /base_waypoints
        msg_type: styx_msgs/Lane
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        styx_msgs/Waypoint[] waypoints
          geometry_msgs/PoseStamped pose
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Pose pose
              geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
              geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
          geometry_msgs/TwistStamped twist
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Twist twist
              geometry_msgs/Vector3 linear
                float64 x
                float64 y
                float64 z
              geometry_msgs/Vector3 angular
                float64 x
                float64 y
                float64 z     
        
        """
        # reserve the waypoint
        self.base_lane = waypoint_msg
        if not self.waypoints_2d:
            # upload data
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoint_msg.waypoints]
        
            # prepare waypoint data for KDTree
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def publish_waypoints(self):
        """        
        Task: Invokes the waypoint publisher and publishes the nearest waypoints to the
              /final_waypoints topic.
        arguments:
        - closest_idx: int, the idx of the nearest waypoints in front of the car.
        ROS integration:
        ===
        Type: Publisher
        Topic: /final_waypoints
        msg_type: styx_msgs/Lane
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        styx_msgs/Waypoint[] waypoints
          geometry_msgs/PoseStamped pose
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Pose pose
              geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
              geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
          geometry_msgs/TwistStamped twist
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Twist twist
              geometry_msgs/Vector3 linear
                float64 x
                float64 y
                float64 z
              geometry_msgs/Vector3 angular
                float64 x
                float64 y
                float64 z
        
        """
        # generate waypoints and update their velocity , based on how we want to car to behave
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        # create lane object
        lane = Lane()

        # get the closest index from our cars
        closest_idx = self.get_nearest_waypoint_idx()

        # get the farthest index which is the closest index plus the number of lookahead waypoints
        farthest_idx = closest_idx + LOOKAHEAD_WPS

        # keep the lane waypoints from our closest index to the farthest index
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
    
        # don't care about it, leave it alone
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        # brake action
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane
   
    def decelerate_waypoints(self, waypoints, closest_idx):
        # don't modify base waypoint directly, so use temp[]
        temp = []

        # enumerate over the sliced list of waypoints
        for i, wp in enumerate(waypoints):
        
            # create new Waypoint object
            p = Waypoint()

            # set the pose to the base waypoint pose
            p.pose = wp.pose
           
            # find the center of the car ,so use "-2"
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
           
            # figure out how far away to decelerate
            dist = self.distance(waypoints, i, stop_idx)
            
            # velocity falling down profile when brake, the larger distance the smaller brake
            vel = math.sqrt(2 * MAX_DECEL * dist)

            # too slow speed , just stop the car
            if vel <1.:
                vel = 0.
        
            # the square root can become very large as the distance is large, we don't want to set a really velocity if we're long way away from the stop waypoint, just keep the velocity that was given
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

    # below is the same as before, not chage.
    def get_nearest_waypoint_idx(self):
        """
        Task: Finds the nearest waypoint according to the car's current position 
              and returns the index of that waypoint
        returns: int, index of nearest waypoint in self.waypoints_2d
        """
    
        x = self.pose_msg.pose.position.x
        y = self.pose_msg.pose.position.y
        # lookup the KDtree to find the nearest point and return its index
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = np.array(self.waypoints_2d[closest_idx])
        prev_coord = np.array(self.waypoints_2d[closest_idx - 1])
        current_pos = np.array([x, y])

        wp_vec = closest_coord - prev_coord
        car_vec = closest_coord - current_pos

        # calculate dot product between the two vectors 
        # to determine if  closest point is ahead of car
        # -> same heading if dot product is > 0
        dot_product = np.dot(wp_vec, car_vec)

        # if the closest point is not ahead of the vehicle, choose the next point
        if dot_product < 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        return closest_idx


    def traffic_cb(self, msg):
        # get the stop line waypoint index
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        # get speed information 
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        # set speed
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
