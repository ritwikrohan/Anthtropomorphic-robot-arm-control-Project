#!/usr/bin/env python3
import rospy
from planar_3dof_control.msg import EndEffector
from geometry_msgs.msg import Vector3
from antropomorphic_project.ik_antropomorphic_arm import compute_ik
from antropomorphic_project.move_joints import JointMover
from antropomorphic_project.rviz_marker import MarkerBasics

class PlanarEndEffectorMover(object):

    def __init__(self, wait_reach_goal=True):
        rospy.init_node('antropomorphic_end_effector_mover')
        self.markerbasics_object = MarkerBasics()
        self.unique_marker_index = 0
        self.robot_mover = JointMover()

        ee_pose_commands_topic = "/ee_pose_commands"
        rospy.Subscriber(ee_pose_commands_topic, EndEffector, self.ee_pose_commands_clb)
        ee_pose_commands_data = EndEffector()
        while ee_pose_commands_data is None and not rospy.is_shutdown():
            try:
                ee_pose_commands_data = rospy.wait_for_message(ee_pose_commands_topic, EndEffector, timeout=0.5)
            except:
                rospy.logwarn("Waiting for first EE command Pose in topic =" + str(ee_pose_commands_topic))
                pass
        
        self.Pee_x = ee_pose_commands_data.ee_xy_theta.x
        self.Pee_y = ee_pose_commands_data.ee_xy_theta.y
        self.chi = ee_pose_commands_data.ee_xy_theta.z

        self.elbow_pol = ee_pose_commands_data.elbow_policy.data        

        end_effector_real_pose_topic = "/end_effector_real_pose"
        rospy.Subscriber(end_effector_real_pose_topic, Vector3, self.end_effector_real_pose_clb)
        end_effector_real_pose_data = Vector3()
        while end_effector_real_pose_data is None and not rospy.is_shutdown():
            try:
                end_effector_real_pose_data = rospy.wait_for_message(end_effector_real_pose_topic, Vector3, timeout=0.5)
            except:
                rospy.logwarn("Waiting for first EE command Pose in topic =" + str(end_effector_real_pose_topic))
                pass
        
        self.Pee_x_real = end_effector_real_pose_data.x
        self.Pee_y_real = end_effector_real_pose_data.y
        self.chi_real = end_effector_real_pose_data.z

        print('PlanarEndEffectorMover started')

    def end_effector_real_pose_clb(self,msg):

        self.Pee_x_real = msg.x
        self.Pee_y_real = msg.y
        self.chi_real = msg.z

        rospy.logdebug("Pxy_Real=["+str(self.Pee_x_real)+","+str(self.Pee_y_real)+"] --- CHI_real="+str(self.chi_real))
        rospy.logdebug("Pxy=["+str(self.Pee_x)+","+str(self.Pee_y)+"] --- CHI="+str(self.chi))

    def ee_pose_commands_clb(self, msg):
        pass

        self.Pee_x = msg.ee_xy_theta.x
        self.Pee_y = msg.ee_xy_theta.y
        self.chi = msg.ee_xy_theta.z

        self.elbow_pol = msg.elbow_policy.data

        r1 = 0.0
        r2 = 1.0
        r3 = 1.0

        DH_parameters={"r1":r1,
            "r2":r2,
            "r3":r3}

        #theta_array, possible_solution = compute_ik(Pee_x=self.Pee_x, Pee_y=self.Pee_y, chi=self.chi,DH_parameters=DH_parameters, elbow_config = self.elbow_pol)
        res = compute_ik([self.Pee_x, self.Pee_y, self.chi])
        for l, possible_solution in res:
            if possible_solution == True:
                break

        if possible_solution:
            theta_1 = l[0]
            theta_2 = l[1]
            theta_3 = l[2]

            self.robot_mover.move_all_joints(theta_1, theta_2, theta_3)
            self.markerbasics_object.publish_point(self.Pee_x, self.Pee_y, self.chi, index=self.unique_marker_index)
            self.unique_marker_index += 1 
        else:
            rospy.logerr("NO POSSIBLE SOLUTION FOUND, Robot Cant reach that pose")

    

def main():

    planar_object = PlanarEndEffectorMover()
    rospy.spin()



if __name__ == '__main__':
    main()