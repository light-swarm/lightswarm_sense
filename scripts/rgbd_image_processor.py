#!/usr/bin/env python
#
#
#

import rospy
import sys
import time
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ros_node import RosNode


class RGBDImageProcessor(RosNode):
    """
    RGBDImageProcessor


    A classy way to process RGB and Depth images coming out
    of a kinect over ros topics. Run it like so:

    improc = RBBDImageProcess("image_processor_node")
    improc.run()

    This class subscribes to a couple of ros topics:

    /camera/rgb/image_color
    /camera/depth_registered/image_raw

    Running openni_launch openni.launch will populate these channels.
    
    The instance will call

    self.process_rgbd_image(rgb_ndarry, depth_ndarray)

    with the latest rgb and depth images converted to a
    convenient numpy.ndarry for opencv2 operations. frequency
    of the call will depend on how fast the processing runs. 
    
    In short: override process_rgbd_image() for a good time

    """
    def __init__(self, node_name):
        super(RGBDImageProcessor, self).__init__(node_name)
        
        self.bridge = CvBridge()

        self._rgb_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self._rgb_callback)
        self._depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self._depth_callback)

        self._rgb_frame_count = 0
        self._depth_frame_count = 0
        self._process_count = 0
        self._start_time = time.time()

        self._its_alive = True

        self.rgb_ndarray = None
        self.image_ndarray = None

        self._cv_window_initialized = False



    def _depth_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # The depth image is a single-channel float32 image
            depth_image = self.bridge.imgmsg_to_cv(ros_image, "32FC1")
        except CvBridgeError, e:
            print e

        self.depth_ndarray = numpy.array(depth_image, dtype=numpy.float32)
        self._depth_frame_count += 1



    def _rgb_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv(ros_image, 'bgr8')
        except CvBridgeError, e:
            print e

        self.rgb_ndarray = numpy.array(frame, dtype=numpy.uint8)
        self._rgb_frame_count += 1


    def _process_rgbd_image(self, rgb_ndarray, depth_ndarray):
        self.process_rgbd_image(rgb_ndarray, depth_ndarray)
        self._process_count += 1




    #############################################################
    # override me
    #
    def process_rgbd_image(self, rgb_ndarray, depth_ndarray):
        self.show_rgbd_image(rgb_ndarray, depth_ndarray)
    #
    #
    #############################################################

    def _draw_status_text(self, image):
        processing_hz = 0
        rgb_hz = 0
        depth_hz = 0
        elapsed_time = time.time() - self._start_time

        if elapsed_time > 0:
            processing_hz = self._process_count / elapsed_time
            rgb_hz = self._rgb_frame_count / elapsed_time
            depth_hz = self._depth_frame_count / elapsed_time


        status_text = 'rgb: %.1fHz depth: %.1fHz' % (rgb_hz, depth_hz)
        cv2.putText(image, 
                    status_text,
                    (10,15), # org(x,y)
                    cv2.FONT_HERSHEY_SIMPLEX, # font_face
                    0.4,  #font_scale
                    (90, 90, 90), 1) # color         

        status_text = 'out: %.1fHz' % (processing_hz)
        cv2.putText(image, 
                    status_text,
                    (10,30), # org(x,y)
                    cv2.FONT_HERSHEY_SIMPLEX, # font_face
                    0.5,  #font_scale
                    (0, 90, 190), 2) # color       


    def show_rgbd_image(self, rgb_ndarray, depth_ndarray):
        self._draw_status_text(rgb_ndarray)

        depth_fake = cv2.cvtColor(depth_ndarray, cv2.COLOR_GRAY2BGR)
        cv2.normalize(depth_fake, depth_fake, 0, 255, cv2.NORM_MINMAX)

        depth_fake = depth_fake.astype(numpy.uint8)

        vis = numpy.hstack((rgb_ndarray, depth_fake))

        self._show_image(vis)

    def _show_image(self, image_ndarray):
        if self._its_alive == False:
            return

        if not self._cv_window_initialized:
            cv2.cv.NamedWindow(self.node_name, cv2.cv.CV_WINDOW_NORMAL)
            self._cv_window_initialized = True

        cv2.imshow(self.node_name, image_ndarray)
        key = cv2.waitKey(1)
        if key == 'q':
            # never seen this work
            rospy.loginfo('got q. shutting down')
            self._its_alive = False


    def cleanup(self):
        self._its_alive = False
        cv2.destroyAllWindows()

    def run(self):
        while self._its_alive:
            if self._depth_frame_count > 0 and self._rgb_frame_count > 0:
               self._process_rgbd_image(self.rgb_ndarray, self.depth_ndarray)








if __name__ == '__main__':
    image_processor = RGBDImageProcessor('test_processor')
    image_processor.run()
