#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Vector3
import time
import numpy as np
from tf.transformations import euler_from_quaternion

# def euler_from_quaternion(quaternion):
#     """
#     Converts quaternion (w in last place) to euler roll, pitch, yaw
#     quaternion = [x, y, z, w]
#     Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
#     """
#     x = quaternion[0]
#     y = quaternion[1]
#     z = quaternion[2]
#     w = quaternion[3]

#     sinr_cosp = 2 * (w * x + y * z)
#     cosr_cosp = 1 - 2 * (x * x + y * y)
#     roll = np.arctan2(sinr_cosp, cosr_cosp)

#     sinp = 2 * (w * y - z * x)
#     pitch = np.arcsin(sinp)

#     siny_cosp = 2 * (w * z + x * y)
#     cosy_cosp = 1 - 2 * (y * y + z * z)
#     yaw = np.arctan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw

class Model1ToModel2Listener(object):

    def __init__(self, follower_model_name, model_to_be_followed_name):

        self._listener = tf.TransformListener()
        # We ad this sleep to allow the listener time to be initialised
        # Otherwise the lookupTransform could give not found frame issue.
        time.sleep(1)

        self._follower_model_name = follower_model_name
        self._model_to_be_followed_name = model_to_be_followed_name

        rospy.loginfo("child frame name ="+str(self._follower_model_name))
        rospy.loginfo("base_ frame name =" +
                      str(self._model_to_be_followed_name))

        self.pub_end_effector_pose = rospy.Publisher("/end_effector_real_pose", Vector3, queue_size=1)

        self._rate = rospy.Rate(10.0)
 

        self._follower_model_frame = "/"+self._follower_model_name
        self._model_to_be_followed_frame = "/"+self._model_to_be_followed_name


    def quat_to_euler(self, orientation_q):
        (roll, pitch, yaw) = euler_from_quaternion(orientation_q)
        return [roll, pitch, yaw]


    def start_loop(self):

        while not rospy.is_shutdown():
            
            try:
                (trans, rot) = self._listener.lookupTransform(
                    self._follower_model_frame, self._model_to_be_followed_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rospy.logdebug("trans ===="+str(trans))
            euler_values = self.quat_to_euler(rot)
            rospy.logdebug("rot ===="+str(euler_values))

            pose_real_msg = Vector3()
            # Because its planar, only Px, Py and Z rotation are not zero
            ndigits = 3
            pose_real_msg.x = round(trans[0], ndigits)
            pose_real_msg.y = round(trans[1], ndigits)
            # We place the yaw value
            pose_real_msg.z = round(euler_values[2], ndigits)

            rospy.logdebug("MSGS TF="+str(pose_real_msg))
            self.pub_end_effector_pose.publish(pose_real_msg)

            self._rate.sleep()


def main():
    rospy.init_node('end_effector_pose_listener')

    child_frame = "P_0"
    base_frame = "P_3"

    listener_obj = Model1ToModel2Listener(
        child_frame, base_frame)

    try:
        listener_obj.start_loop()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()