from styx_msgs.msg import TrafficLight
import os
import tensorflow as tf
import numpy as np
import time


import matplotlib.pyplot as plt

class TLClassifier(object):
    def __init__(self):

        # default status
        self.current_status = TrafficLight.UNKNOWN

        #working directory
        working_dir = os.path.dirname(os.path.realpath(__file__))

        #working model
        self.checkpoint = working_dir + '/frozen_inference_graph.pb'

        # Create a label dictionary
        item_yellow = {'id': 1, 'name': 'Yellow'}
        item_red_left = {'id': 2, 'name': 'RedLeft'}
        item_red = {'id': 3, 'name': 'Red'}
        item_green_left = {'id': 4, 'name': 'GreenLeft'}
        item_green = {'id': 5, 'name': 'Green'}
        item_off = {'id': 6, 'name': 'Off'}
        item_green_right = {'id': 7, 'name': 'GreenRight'}
        item_green_straight = {'id': 8, 'name': 'GreenStraight'}
        item_green_straight_right = {'id': 9, 'name': 'GreenStraightRight'}
        item_red_right = {'id': 10, 'name': 'RedRight'}
        item_red_straight = {'id': 11, 'name': 'RedStraight'}
        item_red_straight_left = {'id': 12, 'name': 'RedStraightLeft'}
        item_green_straight_left = {'id': 13, 'name': 'GreenStraightLeft'}


        self.label_dict = {1: item_yellow, 2: item_red_left, 3: item_red,
                          4: item_green_left, 5: item_green, 6: item_off, 7: item_green_right,
                          8: item_green_straight, 9: item_green_straight_right, 10: item_red_right,
                          11: item_red_straight, 12: item_red_straight_left, 13: item_green_straight_left}

        # Build the model
        self.image_np_output = None
        self.compute_graph = tf.Graph()

        self.current_light = TrafficLight.UNKNOWN

        # create config
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        # Create graph
        with self.compute_graph.as_default():
            graph_def= tf.GraphDef()

            # load trained model
            with tf.gfile.GFile(self.checkpoint, 'rb') as fid:
                graph_def.ParseFromString(fid.read())
                tf.import_graph_def(graph_def, name='')

            self.sess = tf.Session(graph=self.compute_graph, config=config)

        # parameter names of tensorflow
        self.image_tensor = self.compute_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.compute_graph.get_tensor_by_name('detection_boxes:0')
        # scores of detections
        self.detection_scores = self.compute_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.compute_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.compute_graph.get_tensor_by_name('num_detections:0')

        self.activated = True  # flag to turn off classifier dufidring development
        pass

    def get_classification( self, image):

        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Run the classifier if activated flag is true
        if self.activated is True:

            # create image as numpy array
            #image1 = load_image_into_numpy_array(image)
            np_exp_image = np.expand_dims(image, axis=0)
            # get the detections and scores and bounding boxes
            with self.compute_graph.as_default():
                (boxes, scores, classes, num) = self.sess.run( [self.detection_boxes, self.detection_scores,
                                                                self.detection_classes, self.num_detections],
                                                               feed_dict={self.image_tensor: np_exp_image})

            #PATH_TO_LABELS = 'tl_label_map.pbtxt'
            # create np arrays
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)
            num = np.squeeze(num)

            # Set a Classification threshold
            classification_threshold = .5

            print("classification complete")

            # Iterate the boxes to get all detections
            for i in range(boxes.shape[0]):

                # Get class name for detections with high enough scores
                if scores is None or scores[i] > classification_threshold:
                    class_name = self.label_dict[classes[i]]['name']


                    # Set default state to unknown
                    self.current_light = TrafficLight.UNKNOWN

                    print("Class Name is", class_name)

                    if class_name == 'Red':
                        self.current_light = TrafficLight.RED
                    elif class_name == 'Green':
                        self.current_light = TrafficLight.GREEN
                    elif class_name == 'Yellow':
                        self.current_light = TrafficLight.YELLOW


        self.image_np_output = image

        return self.current_light
