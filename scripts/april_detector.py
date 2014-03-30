#!/usr/bin/env python

import rospy
from lightswarm_core.msg import Objects
from lightswarm_core.msg import Cylinder
from april_tag.msg import AprilTagList
from april_tag.msg import AprilTag

WORLD_LIMIT = 100
GHETTO_CALIBRATION_X_OFFSET = 0
GHETTO_CALIBRATION_Z_OFFSET = 280

class AprilDetector(object):
	def __init__(self):
		rospy.init_node('april_detector')
		self.sub = rospy.Subscriber('/april_tags', AprilTagList, self.april_callback)
		self.pub = rospy.Publisher('/objects', Objects)

	def april_callback(self, april_tags):
		objects = Objects()
		for at in april_tags.april_tags:
			cylinder = Cylinder()
			cylinder.location.x = at.x - GHETTO_CALIBRATION_X_OFFSET
			cylinder.location.y = at.z - GHETTO_CALIBRATION_Z_OFFSET
			cylinder.height = 180
			cylinder.radius = 20
			objects.cylinders.append(cylinder)
		self.pub.publish(objects)


	def run(self):
		rospy.spin()


if __name__ == '__main__':
	detector = AprilDetector()
	detector.run()



