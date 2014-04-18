#!/usr/bin/env python

import rospy
from lightswarm_core.msg import Objects
from lightswarm_core.msg import Cylinder
from april_tag.msg import AprilTagList
from april_tag.msg import AprilTag


import numpy as np
import affine
import yaml

WORLD_LIMIT = 100
GHETTO_CALIBRATION_X_OFFSET = 0
GHETTO_CALIBRATION_Z_OFFSET = 202

CONFIG_FILE = 'lightswarm_core/params/config.yaml'

class AprilDetector(object):
	def __init__(self):
		rospy.init_node('april_detector')
		config_filename = rospy.get_param('config_file', CONFIG_FILE)
		self.config_map = yaml.load(open(config_filename))
		self.setup_camera_transform()
		self.pub = rospy.Publisher('/objects', Objects)		
		self.sub = rospy.Subscriber('/april_tags', AprilTagList, self.april_callback)




	def setup_camera_transform(self):
		camera_points = np.asarray(self.config_map.get('sense_camera_points'))
		world_points = np.asarray(self.config_map.get('sense_world_points'))
		self.transform_matrix = affine.affine_matrix_from_points(camera_points.T,
																 world_points.T,
																 shear=False,
																 scale=False)
		print 'got camera points %s' % str(camera_points)
		print 'got world points %s' % str(world_points)

	def transform_camera_coordinates(self, cx, cy, cz):
		wx = cx - GHETTO_CALIBRATION_X_OFFSET
		wy = cz - GHETTO_CALIBRATION_Z_OFFSET
		wz = 0
		wx, wy, wz, junk = self.transform_matrix.dot([cx, cy, cz, 1])
		int_cord = ['%s' % int(x) for x in (cx, cy, cz, wx, wy, wz)]
		print '%s -> %s' % (':'.join(int_cord[:3]), ':'.join(int_cord[3:]))
		return wx, wy, wz



	def april_callback(self, april_tags):
		objects = Objects()
		for at in april_tags.april_tags:
			cylinder = Cylinder()
			x, y, z = self.transform_camera_coordinates(at.x, at.y, at.z)
			cylinder.location.x = x
			cylinder.location.y = y
			cylinder.height = 180
			cylinder.radius = 20
			objects.cylinders.append(cylinder)
		self.pub.publish(objects)


	def run(self):
		rospy.spin()


if __name__ == '__main__':
	detector = AprilDetector()
	detector.run()



