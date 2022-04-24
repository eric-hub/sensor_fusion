#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import sympy


qw=sympy.Symbol('qw')
qx=sympy.Symbol('qx')
qy=sympy.Symbol('qy')
qz=sympy.Symbol('qz')

X=sympy.Symbol('X')
Y=sympy.Symbol('Y')
Z=sympy.Symbol('Z')

row1 = (qw**2+qx**2-qy**2-qz**2)*X+2*(qx*qy-qw*qz)*Y+2*(qx*qz+qw*qy)*Z
row2 = 2*(qx*qy+qw*qz)*X+(qw**2-qx**2+qy**2-qz**2)*Y+2*(qy*qz-qw*qx)*Z
row3 = 2*(qx*qz - qw*qy)*X + 2*(qy*qz+qw*qx)*Y+(qw**2-qx**2-qy**2+qz**2)*Z

J_11 = sympy.diff(row1,qw)
J_12 = sympy.diff(row1,qx)
J_13 = sympy.diff(row1,qy)
J_14 = sympy.diff(row1,qz)

J_21 = sympy.diff(row2,qw)
J_22 = sympy.diff(row2,qx)
J_23 = sympy.diff(row2,qy)
J_24 = sympy.diff(row2,qz)

J_31 = sympy.diff(row3,qw)
J_32 = sympy.diff(row3,qx)
J_33 = sympy.diff(row3,qy)
J_34 = sympy.diff(row3,qz)

print(sympy.simplify(J_11))
print(sympy.simplify(J_12))
print(sympy.simplify(J_13))
print(sympy.simplify(J_14))

print('-------------')

print(sympy.simplify(J_21))
print(sympy.simplify(J_22))
print(sympy.simplify(J_23))
print(sympy.simplify(J_24))

print('-------------')

print(sympy.simplify(J_31))
print(sympy.simplify(J_32))
print(sympy.simplify(J_33))
print(sympy.simplify(J_34))
