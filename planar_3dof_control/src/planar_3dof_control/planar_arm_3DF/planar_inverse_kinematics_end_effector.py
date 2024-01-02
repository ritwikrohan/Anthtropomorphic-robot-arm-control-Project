#!/usr/bin/env python

from math import atan2, pi, sin, cos, pow, sqrt



from dataclasses import dataclass
@dataclass
class EndEffectorWorkingSpace:
    # Pos of The P2, which is the one we resolved for
    Pee_x: float
    Pee_y: float
    # Orientation of the X axis respect frame 0
    Xee_x: float
    Xee_y: float

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
        Pee_x = end_effector_pose.Pee_x
        Pee_y = end_effector_pose.Pee_y

        # The Angle that Xee makes with frames 0 x axis
        Xee_x = end_effector_pose.Xee_x
        Xee_y = end_effector_pose.Xee_y

        # We get all the DH parameters
        r1 = self.get_dh_param("r1")
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        print("Input Data===== ELBOW P1 CONFIG = "+str(elbow_configuration))
        print("Pee_x = "+str(Pee_x))
        print("Pee_y = "+str(Pee_y))
        print("Xee_x = "+str(Xee_x))
        print("Xee_y = "+str(Xee_y))
        print("r1 = "+str(r1))
        print("r2 = "+str(r2))
        print("r3 = "+str(r3))

        # We declare all the equations for theta1, theta2, theta3 and auxiliary
        #########################################################################
        # theta_2
        U = r3 * Xee_x
        W = r3 * Xee_y

        G = ( pow(Pee_x,2) + pow(Pee_y,2) + pow(U,2) + pow(W,2)  - (2.0 * (Pee_x*U + Pee_y*W)) - pow(r1,2) - pow(r2,2)) / (2.0 * r1 * r2)

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
            numerator_2 = Pee_y - W
            denominator_2 = Pee_x - U

            theta_1 = atan2(numerator_2,denominator_2) - atan2(r2*sin(theta_2),(r1 + r2*cos(theta_2)))

            #########################################################################


            #########################################################################
            # theta3
            # 
            theta_3 = atan2(Xee_y, Xee_x) - theta_1 - theta_2
            #########################################################################

            theta_array = [theta_1, theta_2, theta_3]

        return theta_array, possible_solution

def calculate_ik(Pee_x, Pee_y, chi, DH_parameters, elbow_config = "down"):

    Xee_x = cos(chi)
    Xee_y = sin(chi)

    ik = ComputeIk(DH_parameters = DH_parameters)
    end_effector_pose = EndEffectorWorkingSpace(Pee_x = Pee_x,
                                                Pee_y = Pee_y,
                                                Xee_x = Xee_x,
                                                Xee_y = Xee_y)

    thetas, possible_solution = ik.compute_ik(  end_effector_pose=end_effector_pose,
                                                elbow_configuration = elbow_config)

    print("Angles thetas solved ="+str(thetas))
    print("possible_solution = "+str(possible_solution))

    return thetas, possible_solution

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

    Pee_x = 1.0
    Pee_y = 0.0
    chi = -pi/2

    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, chi=chi,DH_parameters=DH_parameters, elbow_config = "down")
    calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, chi=chi,DH_parameters=DH_parameters, elbow_config = "up")

