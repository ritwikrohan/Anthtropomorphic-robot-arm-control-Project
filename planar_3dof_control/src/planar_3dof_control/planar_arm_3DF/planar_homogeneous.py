#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp
from sympy.interactive import printing
from sympy import preview

# To make display prety
printing.init_printing(use_latex = True)

theta_i = Symbol("theta_i")
alpha_i = Symbol("alpha_i")
r_i = Symbol("r_i")
d_i = Symbol("d_i")

DH_Matric_Generic = Matrix([[cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), r_i*cos(theta_i)],
                            [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), r_i*sin(theta_i)],
                            [0, sin(alpha_i), cos(alpha_i), d_i],
                            [0,0,0,1]])

result_simpl = simplify(DH_Matric_Generic)

# Save to local File
preview(result_simpl, viewer='file', filename="out.png", dvioptions=['-D','300'])


# Now we create A01, A12, A23

theta_1 = Symbol("theta_1")
theta_2 = Symbol("theta_2")
theta_3 = Symbol("theta_3")


alpha_planar = 0.0

alpha_1 = alpha_planar
alpha_2 = alpha_planar
alpha_3 = alpha_planar

r_1 = Symbol("r_1")
r_2 = Symbol("r_2")
r_3 = Symbol("r_3")

d_planar = 0.0
d_1 = d_planar
d_2 = d_planar
d_3 = d_planar

A01 = DH_Matric_Generic.subs(r_i,r_1).subs(alpha_i,alpha_1).subs(d_i,d_1).subs(theta_i, theta_1)
A12 = DH_Matric_Generic.subs(r_i,r_2).subs(alpha_i,alpha_2).subs(d_i,d_2).subs(theta_i, theta_2)
A23 = DH_Matric_Generic.subs(r_i,r_3).subs(alpha_i,alpha_3).subs(d_i,d_3).subs(theta_i, theta_3)

A03 = A01 * A12 * A23

A03_simplify = trigsimp(A03)

# We save
preview(A01, viewer='file', filename="A01.png", dvioptions=['-D','300'])
preview(A12, viewer='file', filename="A12.png", dvioptions=['-D','300'])
preview(A23, viewer='file', filename="A23.png", dvioptions=['-D','300'])

preview(A03, viewer='file', filename="A03.png", dvioptions=['-D','300'])
preview(A03_simplify, viewer='file', filename="A03_simplify.png", dvioptions=['-D','300'])

# Added for IK
A02 = A01 * A12
A02_simplify = trigsimp(A02)
preview(A02, viewer='file', filename="A02.png", dvioptions=['-D','300'])
preview(A02_simplify, viewer='file', filename="A02_simplify.png", dvioptions=['-D','300'])


