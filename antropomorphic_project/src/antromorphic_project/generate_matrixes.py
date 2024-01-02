"""
Find the Denavit-Hartenberg parameters for all three joints.
Compute the Homogeneous Matrix for transforming from Frame 0 to Frame 3.
Save all the Homogeneous matrixes, simplified, in png files (as shown in this course).
The matrixes that have to be generated are: A0_1,A1_2,A2_3, A0_3, A1_3.
This script is recomended to be in a class
"""
from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp, N
from sympy.interactive import printing
from sympy import preview
import math

theta_i = Symbol("theta_i")
alpha_i = Symbol("alpha_i")
r_i = Symbol("r_i")
d_i = Symbol("d_i")

class GenerateMatrixes:
    def __init__(self):

        self.DH_Matric_Generic = Matrix([[cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), r_i*cos(theta_i)],
                            [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), r_i*sin(theta_i)],
                            [0, sin(alpha_i), cos(alpha_i), d_i],
                            [0,0,0,1]])
        
        self.init_matrixes()
    

    def init_matrixes(self):
        # Create A01
        theta_1 = Symbol('theta_1')
        r_1 = Symbol('r_1') # 0.0
        d_1 = 0
        alpha_1 = math.radians(90)
        self.A01 = self.generate_frame_matrix(theta_1, alpha_1, r_1, d_1)
        self.A01_simplify = trigsimp(self.A01)

        # Create A12
        theta_2 = Symbol('theta_2')
        r_2 = Symbol('r_2') # 1.0
        d_2 = 0
        alpha_2 = 0

        self.A12 = self.generate_frame_matrix(theta_2, alpha_2, r_2, d_2)
        self.A12_simplify = trigsimp(self.A12)

        # Create A23
        theta_3 = Symbol('theta_3')
        r_3 = Symbol('r_3') # 1.0
        d_3 = 0
        alpha_3 = 0

        self.A23 = self.generate_frame_matrix(theta_3, alpha_3, r_3, d_3)
        self.A23_simplify = trigsimp(self.A23)

        # Create A03
        self.A03 = self.A01 * self.A12 * self.A23
        self.A03_simplify = trigsimp(self.A03)

        # Create A13
        self.A13 = self.A12 * self.A23
        self.A13_simplify = trigsimp(self.A13)
        
    
    def generate_frame_matrix(self, theta, alpha, r, d):
        return self.DH_Matric_Generic.subs(r_i,r).\
                                      subs(alpha_i,alpha).\
                                      subs(d_i,d).\
                                      subs(theta_i, theta)



if __name__=='__main__':
    matrix = GenerateMatrixes()

    print('Generating matrixes...')

    # To make display prety
    printing.init_printing(use_latex = True)

    # # Save to images
    preview(matrix.A01, viewer='file', filename='A01.png', dvioptions=['-D','300'])
    preview(matrix.A12, viewer='file', filename='A12.png', dvioptions=['-D','300'])
    preview(matrix.A23, viewer='file', filename='A23.png', dvioptions=['-D','300'])

    preview(matrix.A03, viewer='file', filename='A03.png', dvioptions=['-D','300'])
    preview(matrix.A13, viewer='file', filename='A13.png', dvioptions=['-D','300'])


    preview(matrix.A01_simplify, viewer='file', filename='A01_simplify.png', dvioptions=['-D','300'])
    preview(matrix.A12_simplify, viewer='file', filename='A12_simplify.png', dvioptions=['-D','300'])
    preview(matrix.A23_simplify, viewer='file', filename='A23_simplify.png', dvioptions=['-D','300'])

    preview(matrix.A03_simplify, viewer='file', filename='A03_simplify.png', dvioptions=['-D','300'])
    preview(matrix.A13_simplify, viewer='file', filename='A13_simplify.png', dvioptions=['-D','300'])

    print('Generating finished.')



