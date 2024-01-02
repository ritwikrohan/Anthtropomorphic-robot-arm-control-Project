#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from planar_3dof_control.msg import EndEffector
from math import sin, cos, pi

class EE_Client(object):

    def __init__(self):

        
        self.pub_end_effector_commads= rospy.Publisher("/ee_pose_commands", EndEffector, queue_size=1)

        self._rate = rospy.Rate(10.0)

        self.unitary_angle = 0.0
        self.a = 2.0
        self.b = 1.0
 

    def generate_elipse_points(self, delta=0.05):

        x = self.a * cos(self.unitary_angle)
        y = self.b * sin(self.unitary_angle)

        self.unitary_angle += delta
        if self.unitary_angle >= 2*pi:
            self.unitary_angle = 0.0

        if x >= 0 and y >= 0:
            quadrant = 1
        elif x < 0 and y >= 0:
            quadrant = 2
        elif x < 0 and y < 0:
            quadrant = 3
        elif x >= 0 and y < 0:
            quadrant = 4
        else:
            assert False, "This is not possible = "+str(x)+","+str(y)
        
        
        

        return x, y, quadrant

        
    def start_loop(self):

        while not rospy.is_shutdown():
            
            x_elipse, y_elipse, quadrant = self.generate_elipse_points()

            ee_msg = EndEffector()

            ee_pose = Vector3()
            ee_pose.x = x_elipse
            ee_pose.y = y_elipse
            ee_pose.z = self.unitary_angle

            # We decide elboy policy based on the quadrant we are in
            if quadrant == 1 or quadrant == 3:
                elbow_policy = "down"
            else:
                elbow_policy = "up"

            ee_msg.ee_xy_theta = ee_pose
            ee_msg.elbow_policy.data = elbow_policy

            print("Elipse Point="+str(ee_pose)+", elbow="+str(elbow_policy))
            self.pub_end_effector_commads.publish(ee_msg)

            self._rate.sleep()


def main():
    rospy.init_node('circular_motion_ee')

    
    ee_client_obj = EE_Client()

    try:
        ee_client_obj.start_loop()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()