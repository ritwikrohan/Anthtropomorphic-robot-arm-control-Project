#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import time

"""
Topics To Write on:
type: std_msgs/Float64
/planar_3dof/joint1_position_controller/command
/planar_3dof/joint2_position_controller/command
/planar_3dof/joint3_position_controller/command

"""

class JointMover(object):

    def __init__(self):

        rospy.loginfo("JointMover Initialising...")
        self.pub_joint1 = rospy.Publisher('/planar_3dof/joint1_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.pub_joint2 = rospy.Publisher('/planar_3dof/joint2_position_controller/command',
                                                             Float64,
                                                             queue_size=1)
        self.pub_joint3 = rospy.Publisher('/planar_3dof/joint3_position_controller/command',
                                                           Float64,
                                                           queue_size=1)

        self.check_connection()

    def move_all_joints(self, theta_1, theta_2, theta_3):
        theta_1_msg = Float64()
        theta_1_msg.data = theta_1
        theta_2_msg = Float64()
        theta_2_msg.data = theta_2
        theta_3_msg = Float64()
        theta_3_msg.data = theta_3
        self.pub_joint1.publish(theta_1_msg)
        self.pub_joint2.publish(theta_2_msg)
        self.pub_joint3.publish(theta_3_msg)

        rospy.logwarn("Moving to Angles= theta_1="+str(theta_1)+", theta_2="+str(theta_2)+", theta_3="+str(theta_3))


    def check_topic_connection(self, publisher_object, topic_name):

        self.rate = rospy.Rate(10)  # 10hz
        conections_joint_1 = publisher_object.get_num_connections()
        rospy.logdebug(conections_joint_1)
        while conections_joint_1 == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to "+str(topic_name)+" yet so we wait and try again")

            conections_joint_1 = publisher_object.get_num_connections()

            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logdebug("ROSInterruptException Triggered")
            

        rospy.logdebug(str(topic_name)+" Publisher Connected")

    def check_connection(self):
        """
        Checks publisher is working
        :return:
        """
        self.check_topic_connection(self.pub_joint1, "pub_joint1")
        self.check_topic_connection(self.pub_joint2, "pub_joint2")
        self.check_topic_connection(self.pub_joint3, "pub_joint3")

        rospy.logdebug("All Publishers READY")



if __name__ == "__main__":
    rospy.init_node('move',log_level=rospy.DEBUG)
    obj = JointMover()
    theta_pos_1 = [1.57 , -1.57 , -1.57]
    theta_pos_2 = [0.0 , 1.57 , -3.1416]
    i = 1
    while not rospy.is_shutdown():
        if i == 1:
            obj.move_all_joints(theta_pos_1[0],theta_pos_1[1],theta_pos_1[2])
            i = 2
        else:
            obj.move_all_joints(theta_pos_2[0],theta_pos_2[1],theta_pos_2[2])
            i = 1
        rospy.loginfo("Sleep...")
        time.sleep(5)
        rospy.loginfo("Sleep...END")

    
