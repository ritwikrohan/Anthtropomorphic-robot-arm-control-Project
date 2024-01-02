#!/usr/bin/env python

from math import atan2, sin, cos, pow, sqrt



from dataclasses import dataclass
@dataclass
class EndEffectorWorkingSpace:
    # Pos of The P2, which is the one we resolved for
    P2_x: float
    P2_y: float
    # Orientation of the X axis respect frame 0, which is regulated by Chi
    chi: float

class ComputeIk():

    def __init__(self, DH_parameters):
        
        # DH parameters
        self.DH_parameters_ = DH_parameters

    def get_dh_param(self, name):

        if name in self.DH_parameters_:
            return self.DH_parameters_[name]
        else:
            assert False, "Asked for Non existen param DH name ="+str(name)

    def compute_ik(self, end_effector_pose, elbow_configuration = "down"):
        
        # Initialization
        p2_x = end_effector_pose.P2_x
        p2_y = end_effector_pose.P2_y

        # The Angle that Xee makes with frames 0 x axis
        chi = end_effector_pose.chi

        # We get all the DH parameters
        r1 = self.get_dh_param("r1")
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        print("Input Data===== ELBOW P1 CONFIG = "+str(elbow_configuration))
        print("p2_x = "+str(p2_x))
        print("p2_y = "+str(p2_y))
        print("chi = "+str(chi))
        print("r1 = "+str(r1))
        print("r2 = "+str(r2))

        # We declare all the equations for theta1, theta2, theta3 and auxiliary
        #########################################################################
        # theta_2
        G = ( pow(p2_x,2) + pow(p2_y,2) - pow(r1,2) - pow(r2,2)) / (2.0 * r1 * r2)

        print(G)

        ## WE HAVE TO CHECK THAT ITS POSSIBLE
        ## -1 <= G <= 1
        possible_solution = False
        if (G <= 1 ) and ( G>= -1):
            possible_solution = True
        else:
            pass
        

        theta_array = []

        if possible_solution:
            # We have to decide which solution we want
            if elbow_configuration == "down":
                # Positive
                numerator_1 = sqrt(1-pow(G,2))
            else:
                numerator_1 = -1.0 * sqrt(1-pow(G,2))

            denominator_1 = G

            theta_2 = atan2(numerator_1, denominator_1)
            #########################################################################


            #########################################################################
            # theta1
            theta_1 = atan2(p2_y,p2_x) - atan2(r2*sin(theta_2),(r1 + r2*cos(theta_2)))

            #########################################################################


            #########################################################################
            # theta3
            # 
            theta_3 = chi - theta_1 - theta_2
            #########################################################################

            theta_array = [theta_1, theta_2, theta_3]

        return theta_array, possible_solution

def calculate_ik(P2_x, P2_y, chi, DH_parameters, elbow_config = "down"):
    ik = ComputeIk(DH_parameters = DH_parameters)
    end_effector_pose = EndEffectorWorkingSpace(P2_x = P2_x,
                                                P2_y = P2_y,
                                                chi = chi)

    thetas, possible_solution = ik.compute_ik(  end_effector_pose=end_effector_pose,
                                                elbow_configuration = elbow_config)

    print("Angles thetas solved ="+str(thetas))
    print("possible_solution = "+str(possible_solution))

if __name__ == '__main__':
    

    r1 = 1.0
    r2 = 1.0
    r3 = 1.0

    # theta_i here are valriables of the joints
    # We only fill the ones we use in the equations, the others were already 
    # replaced in the Homogeneous matrix
    DH_parameters={"r1":r1,
                    "r2":r2,
                    "r3":r3}

    P2_x = 1.0
    P2_y = 1.0
    chi = 0.7

    calculate_ik(P2_x=P2_x, P2_y=P2_y, chi=chi,DH_parameters=DH_parameters, elbow_config = "down")
    calculate_ik(P2_x=P2_x, P2_y=P2_y, chi=chi,DH_parameters=DH_parameters, elbow_config = "up")

