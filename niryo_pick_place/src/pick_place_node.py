#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Empty  # Import the necessary message type for the new topic
from pyniryo import *

class NiryoPickPlaceNode:
    def __init__(self):
        rospy.init_node('niryo_pick_place_node')
        self.sub_niryo_control = rospy.Subscriber('/niryo_control', Bool, self.niryoControlCallback, queue_size=1)
        self.pub_niryo_done = rospy.Publisher('/niryo_done', Empty, queue_size=1)  # Create a publisher for the new topic

    def niryoControlCallback(self, msg):
        # Callback function to handle Niryo control messages
        self.stopped = msg.data
        if self.stopped:
            self.performPickAndPlace()
            # Once pick and place is done, publish a message to inform the TurtleBot
            self.publishDoneMessage()

    def performPickAndPlace(self):
        robot = NiryoRobot("192.168.0.101")

        try:
            robot.calibrate_auto()
            # Move to an observation position then
            robot.move_pose(*[0.001, -0.213, 0.217, 3.1, 1.395, 1.559])
            # Try to do a vision pick:
            if robot.vision_pick('default_workspace_turltelbot', 0/1000.0, ObjectShape.CIRCLE, ObjectColor.RED)[0]:
                # If an object has been taken, do:
                robot.place_from_pose(*[0.326, -0.015, 0.314, -2.232, 1.471, -2.234])
            robot.move_pose(*[0.326, -0.015, 0.364, -2.175, 1.476, -2.178])
            robot.move_pose(*[0, -0.284, 0.325, 2.928, 1.346, 1.383])

        except NiryoRosWrapperException as e:
            rospy.logerr(str(e))

        robot.close_connection()

    def publishDoneMessage(self):
        rospy.loginfo("Niryo is done, publishing message to TurtleBot.")
        # Publish an empty message to indicate that Niryo is done
        self.pub_niryo_done.publish(Empty())

if __name__ == '__main__':
    try:
        node = NiryoPickPlaceNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass

