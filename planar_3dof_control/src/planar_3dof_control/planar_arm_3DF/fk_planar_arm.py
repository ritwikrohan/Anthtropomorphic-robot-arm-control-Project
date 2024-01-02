#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp
from sympy.interactive import printing
from sympy import preview
from math import pi

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

##############
# THETA VALUES
##############

theta_1_val = pi/6
theta_2_val = pi/6
theta_3_val = pi/6

r_1_val = 1.0
r_2_val = 1.0
r_3_val = 1.0

A03_evaluated = A03_simplify.subs(theta_1,theta_1_val).subs(theta_2,theta_2_val).subs(theta_3, theta_3_val).subs(r_1, r_1_val).subs(r_2, r_2_val).subs(r_3, r_3_val)

# We save
preview(A03_simplify, viewer='file', filename="A03_simplify.png", dvioptions=['-D','300'])
preview(A03_evaluated, viewer='file', filename="A03_evaluated.png", dvioptions=['-D','300'])



