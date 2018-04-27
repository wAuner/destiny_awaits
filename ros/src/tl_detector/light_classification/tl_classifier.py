from styx_msgs.msg import TrafficLight
import os
import tensorflow as tf
import numpy as np
import time

class TLClassifier(object):
    def __init__(self):

        # set default status
        self.current_status = TrafficLight.UNKNOWN
        self.current_light = TrafficLight.UNKNOWN

        self.labels = {1: 'Green', 2: 'Red', 3: 'Yellow', 4: 'off'}

        # Prepare the model
        self.detection_graph = tf.Graph()
        self.path_inference_graph = os.path.join(os.path.dirname(os.path.realpath(__file__)), 
                                                    'frozen_inference_graph.pb')        

        # config to avoid cudnn bug
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        # Load Model
        with self.detection_graph.as_default():
            graph_def= tf.GraphDef()

            # load trained model
            with tf.gfile.GFile(self.path_inference_graph, 'rb') as fid:
                graph_def.ParseFromString(fid.read())
                tf.import_graph_def(graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)
        
        self.tensors = {'image_tensor': self.detection_graph.get_tensor_by_name('image_tensor:0'),
                        'detection_boxes': self.detection_graph.get_tensor_by_name('detection_boxes:0'),
                        'detection_scores': self.detection_graph.get_tensor_by_name('detection_scores:0'),
                        'detection_classes': self.detection_graph.get_tensor_by_name('detection_classes:0'),
                        'num_detections': self.detection_graph.get_tensor_by_name('num_detections:0')}
        

    def predict( self, image):

        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """        
        scores, classes= self.sess.run([self.tensors['detection_scores'], self.tensors['detection_classes']],
                                                    feed_dict={self.tensors['image_tensor']: np.expand_dims(image, axis=0)})

        detection_threshold = .5
        self.current_light = TrafficLight.UNKNOWN

        detections = scores[scores > detection_threshold]
        if detections.size >= 1:
            class_name = self.labels[classes[0][0]]
            print 'Detected traffic light: {} prob: {:.2f}'.format(class_name, detections[0])

            if class_name == 'Red':
                self.current_light = TrafficLight.RED
            elif class_name == 'Green':
                self.current_light = TrafficLight.GREEN
            elif class_name == 'Yellow':
                self.current_light = TrafficLight.YELLOW                       

        return self.current_light
