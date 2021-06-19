#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool


class ManualController():
    def __init__(self):
        # ROS init
        rospy.init_node('manual_controller')
        self.rate = rospy.Rate(1.0)

        # Pub
        self.pub_mode = rospy.Publisher('/auto_mode', Bool, queue_size=5)
    
    def publish_mode(self, mode):
        """
        Publish mode
        """
        mode_msg = Bool()
        mode_msg.data = mode
        self.pub_mode.publish(mode_msg)

def main():
    # Define controller
    manual_control = ManualController()

    while not rospy.is_shutdown():
        manual_control.publish_mode(False)
        manual_control.rate.sleep()


if __name__ == '__main__':
    main()