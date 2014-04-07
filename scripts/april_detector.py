#!/usr/bin/env python

import rospy
from lightswarm_core.msg import Objects
from lightswarm_core.msg import Cylinder
from april_tag.msg import AprilTagList
from april_tag.msg import AprilTag

import numpy as np
import affine

WORLD_LIMIT = 100
GHETTO_CALIBRATION_X_OFFSET = 0
GHETTO_CALIBRATION_Z_OFFSET = 202


camera_points = np.float32(np.array([[0,  20, 180], [100,   20, 180], [0, 20, 280], [100, 20, 280]]))
world_points = np.float32(np.array([[0, -100,  20], [100, -100,  20], [0,  0,  20], [100,  0,  20]]))

class AprilDetector(object):
	def __init__(self):
		rospy.init_node('april_detector')
		self.sub = rospy.Subscriber('/april_tags', AprilTagList, self.april_callback)
		self.pub = rospy.Publisher('/objects', Objects)
		self.setup_camera_transform()

	def setup_camera_transform(self):
		#retval, matrix, inliners = cv2.estimateAffine3D(camera_points, world_points)
		#assert retval == 0, 'camera transform failed'

		self.transform_matrix = affine.affine_matrix_from_points(camera_points.T,
																 world_points.T,
																 shear=False,
																 scale=False)

	def transform_camera_coordinates(self, cx, cy, cz):
		wx = cx - GHETTO_CALIBRATION_X_OFFSET
		wy = cz - GHETTO_CALIBRATION_Z_OFFSET
		wz = 0
		#wx, wy, wz, junk = self.transform_matrix.dot([cx, cy, cz, 1])
		#int_cord = ['%s' % int(x) for x in (cx, cy, cz, wx, wy, wz)]
		#print '%s -> %s' % (':'.join(int_cord[:3]), ':'.join(int_cord[3:]))
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



