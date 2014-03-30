#!/usr/bin/env python
#

import cv2
import rospy
import numpy as np
import colorsys
import operator
import math
from rgbd_image_processor import RGBDImageProcessor

from lightswarm_core.msg import Objects
from lightswarm_core.msg import Cylinder


GHETTO_CALIBRATION_X_OFFSET = 0
GHETTO_CALIBRATION_Z_OFFSET = 280

def rgb_by_index(idx):
    idx = idx  % 1.0
    r,g,b = colorsys.hsv_to_rgb(idx, 0.5, 0.5)
    return (int(r*255), int(g*255), int(b*255))

class Blob(object):
    def __init__(self, contour, idx=0):
        self.contour = contour
        self.area = cv2.contourArea(contour)
        self.moments = cv2.moments(contour)
        self.centroid_x = int(self.moments['m10']/(self.moments['m00'] + 1.0))
        self.centroid_y = int(self.moments['m01']/(self.moments['m00'] + 1.0))

        self.median_depth = None
        self.idx = idx

        self.x_w, self.y_w, self.z_w = None, None, None


    def draw(self, image):
        cv2.drawContours(image, [self.contour], 
                        0, # draw the only contour
                        color = rgb_by_index(self.idx), 
                        thickness = -1, # filled
                        lineType = cv2.CV_AA)
        cv2.ellipse(image, box=((self.centroid_x, self.centroid_y), (6,6), 0), color=(0,0,255),
                    thickness = -1)
        if self.z_w is not None:
            cv2.putText(image, "%s %s %s" % (int(self.x_w), int(self.y_w), int(self.z_w)),
                        (self.centroid_x + 5, self.centroid_y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, # font_scale
                        (0, 100, 180), # color
                        2 # thickness
                        )
            cv2.putText(image, "d:%s a:%s" % (int(self.median_depth), int(self.area)),
                        (self.centroid_x + 5, self.centroid_y + 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, # font_scale
                        (0, 100, 180), # color
                        2 # thickness
                        )            

    def _get_bool_mask(self, image):
        mask = np.zeros_like(image)
        mask = mask.astype(np.uint8)
        cv2.drawContours(mask, [self.contour], 
                        0, # draw the only contour
                        color = 1, 
                        thickness = -1, # filled
                        lineType = cv2.CV_AA)
        mask = mask.astype(bool)
        return mask

    def set_world_coordinates_from_depth(self, depth):
        mask = self._get_bool_mask(depth)
        self.median_depth = max(np.median(depth[mask]), 500)

        x_p = -1*(self.centroid_x-320)
        y_p = -1*(self.centroid_y-240)
        f_p = 425

        self.median_depth = self.median_depth / 10 # depth was in mm


        self.x_w = (x_p * self.median_depth) / (math.sqrt((x_p * x_p) + (f_p*f_p)))

        self.y_w = (y_p * self.median_depth) / (math.sqrt((y_p * y_p) + (f_p*f_p)))
         
        self.z_w = math.sqrt((self.median_depth*self.median_depth) 
                              - ((self.x_w*self.x_w) 
                              + (self.y_w * self.y_w)))        



class ForeGrounder(RGBDImageProcessor):

    def __init__(self, node_name):
        super(ForeGrounder, self).__init__(node_name)
        self.pub = rospy.Publisher('/objects', Objects)

        self.fgbg = cv2.BackgroundSubtractorMOG(history=5, nmixtures=3, backgroundRatio=0.7)

    def get_depth_fgmask(self, d):
        depth = d.copy()
        cv2.normalize(depth, depth, 0, 255, cv2.NORM_MINMAX)
        depth = depth.astype(np.uint8)

        fgmask = self.fgbg.apply(depth)

        kernel = np.ones((5,5), np.uint8)
        fgmask = cv2.erode(fgmask, kernel, iterations = 2)
        fgmask = cv2.dilate(fgmask, kernel, iterations = 1)

        return fgmask        

    def find_blobs(self, mask):
        contours0 = cv2.findContours( mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cv2.approxPolyDP(cnt, 3, True) for cnt in contours0[0]]        

        blobs = [Blob(contour=c) for c in contours]
        blobs = [b for b in blobs if b.area > 800] # filter
        blobs = sorted(blobs, key=operator.attrgetter('area'), reverse=True)
        for i,b in enumerate(blobs):
            b.idx = i / 20.0
        return blobs
        #return cv2.findContours(mask, mode=cv2.cv.CV_RETR_LIST, method=cv2.cv.CV_CHAIN_APPROX_SIMPLE)

    def process_rgbd_image(self, rgb, depth):
        fgmask = self.get_depth_fgmask(depth)
        blobs = self.find_blobs(fgmask)
        for blob in blobs:
            blob.set_world_coordinates_from_depth(depth)
            blob.draw(rgb)

        self.publish_obstacles(blobs)

        #rgb_masked = cv2.bitwise_and(rgb, rgb, mask = fgmask)
        cv2.imshow('frame', rgb)
        k = cv2.waitKey(3)

    def publish_obstacles(self, blobs):
        # just publish one obstacle
        if len(blobs) > 0 and blobs[0].z_w is not None:
            objects = Objects()
            cylinder = Cylinder()
            cylinder.location.x = blobs[0].x_w - GHETTO_CALIBRATION_X_OFFSET
            cylinder.location.y = blobs[0].z_w - GHETTO_CALIBRATION_Z_OFFSET
            cylinder.height = 180
            cylinder.radius = 20
            objects.cylinders.append(cylinder)
            self.pub.publish(objects)            




if __name__ == '__main__':
    fg = ForeGrounder('fore_grounder')
    fg.run()
