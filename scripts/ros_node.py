import rospy

class RosNode(object):
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.node_name = node_name

        rospy.on_shutdown(self.cleanup)

    def cleanup(self):
        rospy.loginfo(self.node_name + ' dying')

    def run(self):
        rospy.spin()

    def test():
        pass





