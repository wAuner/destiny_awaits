#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):

        # set log_level to DEBUG mode in order to log traffic light ground truth
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)
        
        # stores the raw pose message
        self.pose = None
        # stores the raw waypoint message of type styx_msgs/Lane
        self.waypoints = None
        # stores the raw camera image message of type styx_msgs/TrafficLightArray
        self.camera_image = None
        # stores the raw traffic light message of type styx_msgs/Image
        self.lights = []
        # flag that is set to true when firts images come in
        self.has_image = True

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        # state of the upcomming traffic light
        self.state = TrafficLight.UNKNOWN
        # state of the previous traffic light
        self.last_state = TrafficLight.UNKNOWN
        # stores coords of the stop line in front of last traffic lights
        self.last_wp = -1 # is set to -1 in case the traffic light is not red
        self.state_count = 0

        # contains a list of (x,y) tuples for all waypoints
        self.waypoints_2d = None
        # KD tree of the x,y waypoints to increase lookup time
        self.waypoint_tree = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

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
        self.base_waypoints_msg = waypoint_msg
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoint_msg.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (TrafficLightArray): Array of TrafficLight objects

        ROS integration:
        ===
        Type: Callback
        Topic: /vehicle/traffic_lights
        msg_type: styx_msgs/TrafficLightArray

        Header header
        styx_msgs/TrafficLight[] lights
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
            uint8 state
                uint8 UNKNOWN=4
                uint8 GREEN=2
                uint8 YELLOW=1
                uint8 RED=0
        """
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        ROS integration:
        ===
        Type: Callback
        Topic: /image_color
        msg_type: sensor_msgs.msg/Image
        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            
            # log simulator ground truth
            if state == 0:
                log_state = "red"
            elif state == 1:
                log_state = "yellow"
            else:
                log_state = "green"
            rospy.logdebug('next traffic light idx %s, state: %s', 
               light_wp, log_state)
            
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        x = pose.position.x
        y = pose.position.y

        # lookup the KDtree to find the nearest point and return its index
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # For testing just return the light state provided by the simulator
        return light.state

        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False
        #
        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose)

            # Find the closest visible traffic light (if one exists)
            diff = len(self.waypoints_2d)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(light.pose.pose)
                # find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
