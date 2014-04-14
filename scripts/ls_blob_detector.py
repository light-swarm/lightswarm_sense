#!/usr/bin/env python

import rospy
from lightswarm_core.msg import Objects
from lightswarm_core.msg import Cylinder
from blob_detector.msg import Blobs
from blob_detector.msg import Blob

import numpy as np
import affine
import yaml

CONFIG_FILE = 'lightswarm_core/params/config.yaml'


class LSBlobDetector(object):
	def __init__(self):
		rospy.init_node('april_detector')
		self.sub = rospy.Subscriber('/blobs', Blobs, self.blobs_callback)
		self.pub = rospy.Publisher('/objects', Objects)

		config_filename = rospy.get_param('config_file', CONFIG_FILE)
		self.config_map = yaml.load(open(config_filename))
		self.setup_camera_transform()

	def setup_camera_transform(self):
		#retval, matrix, inliners = cv2.estimateAffine3D(camera_points, world_points)
		#assert retval == 0, 'camera transform failed'
		camera_points = np.asarray(self.config_map.get('sense_camera_points'))
		world_points = np.asarray(self.config_map.get('sense_world_points'))
		self.transform_matrix = affine.affine_matrix_from_points(camera_points.T,
																 world_points.T,
																 shear=False,
																 scale=False)
		print 'got camera points %s' % str(camera_points)
		print 'got world points %s' % str(world_points)

	def transform_camera_coordinates(self, cx, cy, cz):
		wx, wy, wz, junk = self.transform_matrix.dot([cx, cy, cz, 1])
		int_cord = ['%s' % int(x) for x in (cx, cy, cz, wx, wy, wz)]
		print '%s -> %s' % (':'.join(int_cord[:3]), ':'.join(int_cord[3:]))
		return wx, wy, wz



	def blobs_callback(self, blobs):
		objects = Objects()
		for b in blobs.blobs:
			cylinder = Cylinder()
			x, y, z = self.transform_camera_coordinates(b.x, b.y, b.z)
			cylinder.location.x = x
			cylinder.location.y = y
			cylinder.height = 180
			cylinder.radius = 20
			objects.cylinders.append(cylinder)
		self.pub.publish(objects)


	def run(self):
		rospy.spin()


if __name__ == '__main__':
	detector = LSBlobDetector()
	detector.run()




