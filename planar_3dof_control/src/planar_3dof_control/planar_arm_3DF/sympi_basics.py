from sympy.solvers import solve
from sympy import Symbol

x = Symbol('x')
result = solve(x**2 - 1, x)
print(result)

result = solve(x**2 -x -1, x)
print(result)

from sympy import symbols, trigsimp, sin, cos

x, y, z = symbols('x y z')
result = trigsimp(sin(x)*cos(y) + sin(y)*cos(x))
print(result)

from sympy import Matrix, simplify
from IPython.display import display

theta_i = Symbol("theta_i")
alpha_i1 = Symbol("alpha_i1")
a_i1 = Symbol("a_i1")
d_i = Symbol("d_i")


TB_1 = Matrix([[cos(theta_i), sin(theta_i), 0, a_i1*cos(theta_i)],
          [sin(theta_i), -cos(theta_i), 0, a_i1*sin(theta_i)],
          [0, 0, cos(alpha_i1), d_i],
          [0,0,0,1]])

display(TB_1)
display(simplify(TB_1))

import math

theta_1 = Symbol("theta_1")
alpha_11 = Symbol("alpha_11")
a_11 = Symbol("a_11")
d_1 = Symbol("d_1")

TB_1_sim = TB_1.subs(a_i1,0).subs(alpha_i1,-math.pi).subs(d_i,0.1564).subs(theta_i, theta_1)

display(TB_1_sim)

