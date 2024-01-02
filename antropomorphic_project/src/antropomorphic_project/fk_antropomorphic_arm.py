# #! /usr/bin/env python3
# from sympy import preview, N
# from antropomorphic_project.generate_matrixes import GenerateMatrixes


# if __name__ == '__main__':
#     theta_1 = input('Please enter theta_1: ')
#     theta_2 = input('Please enter theta_2: ')
#     theta_3 = input('Please enter theta_3: ')
#     # theta_1 = 2.356194490192345
#     # theta_2 = 0.5074842211955768
#     # theta_3 = -2.2459278597319283
#     matrix = GenerateMatrixes()

#     r_1 = 0.0
#     r_2 = 1.0
#     r_3 = 1.0
    
#     A03 = matrix.A03_simplify

#     A03_new = A03.subs('theta_1', theta_1)\
#                                 .subs('theta_2', theta_2)\
#                                 .subs('theta_3', theta_3)

#     A03_new = A03_new.subs('r_1', r_1)\
#                                     .subs('r_2', r_2)\
#                                     .subs('r_3', r_3)
    
#     A03_forward_kinematics = N(A03_new)

#     # preview(A03_forward_kinematics, viewer='file', filename='A03_forward_kinematics.png', dvioptions=['-D','300'])
#     print('position: ', A03_forward_kinematics[0:3, -1])
#     print('orientation: ', A03_forward_kinematics[0:3, 0:3])

    