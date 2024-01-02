import math
from sympy import *
from sympy.interactive import printing



printing.init_printing(use_latex = True)

theta_i = Symbol("theta_i")
alpha_i1 = Symbol("alpha_i1")
a_i1 = Symbol("a_i1")
d_i = Symbol("d_i")

theta_1 = Symbol("theta_1")
theta_2 = Symbol("theta_2")
theta_3 = Symbol("theta_3")
theta_4 = Symbol("theta_4")
theta_5 = Symbol("theta_5")
theta_6 = Symbol("theta_6")
theta_7 = Symbol("theta_7")


TB_1 = Matrix([[cos(theta_i), sin(theta_i), 0, a_i1*cos(theta_i)],
          [sin(theta_i), -cos(theta_i), 0, a_i1*sin(theta_i)],
          [0, 0, cos(alpha_i1), d_i],
          [0,0,0,1]])


T1_2 = Matrix([[cos(theta_i), 0, sin(theta_i), a_i1*cos(theta_i)],
          [sin(theta_i), 0, -cos(theta_i), a_i1*sin(theta_i)],
          [0, sin(alpha_i1), 0, d_i],
          [0,0,0,1]])


T2_3 = Matrix([[cos(theta_i), 0, -sin(theta_i), a_i1*cos(theta_i)],
          [sin(theta_i), 0, cos(theta_i), a_i1*sin(theta_i)],
          [0, sin(alpha_i1), 0, d_i],
          [0,0,0,1]])

T3_4 = Matrix([[cos(theta_i), 0, sin(theta_i), a_i1*cos(theta_i)],
          [sin(theta_i), 0, -cos(theta_i), a_i1*sin(theta_i)],
          [0, sin(alpha_i1), 0, d_i],
          [0,0,0,1]])

T4_5 = Matrix([[cos(theta_i), 0, -sin(theta_i), a_i1*cos(theta_i)],
          [sin(theta_i), 0, cos(theta_i), a_i1*sin(theta_i)],
          [0, sin(alpha_i1), 0, d_i],
          [0,0,0,1]])

T5_6 = Matrix([[cos(theta_i), 0, sin(theta_i), a_i1*cos(theta_i)],
          [sin(theta_i), 0, -cos(theta_i), a_i1*sin(theta_i)],
          [0, sin(alpha_i1), 0, d_i],
          [0,0,0,1]])

T6_7 = Matrix([[cos(theta_i), 0, -sin(theta_i), a_i1*cos(theta_i)],
          [sin(theta_i), 0, cos(theta_i), a_i1*sin(theta_i)],
          [0, sin(alpha_i1), 0, d_i],
          [0,0,0,1]])



TB_1_sim = TB_1.subs(a_i1,0).subs(alpha_i1,-math.pi).subs(d_i,0.1564).subs(theta_i, theta_1)
T1_2_sim = T1_2.subs(a_i1,0.0054).subs(alpha_i1,math.pi/2).subs(d_i,0.1284).subs(theta_i, theta_2)
T2_3_sim = T2_3.subs(a_i1,0.0064).subs(alpha_i1,-math.pi/2).subs(d_i,0.2104).subs(theta_i, theta_3)
T3_4_sim = T3_4.subs(a_i1,0.0064).subs(alpha_i1,math.pi/2).subs(d_i,0.2104).subs(theta_i, theta_4)
T4_5_sim = T4_5.subs(a_i1,0.0064).subs(alpha_i1,-math.pi/2).subs(d_i,0.2084).subs(theta_i, theta_5)
T5_6_sim = T5_6.subs(a_i1,0).subs(alpha_i1,math.pi/2).subs(d_i,0.1059).subs(theta_i, theta_6)
T6_7_sim = T6_7.subs(a_i1,0).subs(alpha_i1,-math.pi/2).subs(d_i,0.1059).subs(theta_i, theta_7)



TM = TB_1_sim * T1_2_sim * T2_3_sim * T3_4_sim * T4_5_sim * T5_6_sim * T6_7_sim



display(simplify(TM))
#display(TT)