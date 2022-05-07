#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import sympy

# sympy.init_printing()
# a,b,c,d,e,f,g,h,i = sympy.symbols('a:i')
# A = sympy.Matrix([[a,b,c],[d,e,f]])
# print(sympy.latex(A))
Kax = sympy.Symbol('Kax')
Kay = sympy.Symbol('Kay')
Kaz = sympy.Symbol('Kaz')

Sayx = sympy.Symbol('Sayx')
Sazx = sympy.Symbol('Sazx')
Sazy = sympy.Symbol('Sazy')

Ax = sympy.Symbol('Ax')
Ay = sympy.Symbol('Ay')
Az = sympy.Symbol('Az')

bax = sympy.Symbol('bax')
bay = sympy.Symbol('bay')
baz = sympy.Symbol('baz')

KSa = sympy.Matrix([[Kax,0,0],[Sayx*Kax,Kay,0],[Sazx*Kax,Sazy*Kay,Kaz]])
A=sympy.Matrix([[Ax],[Ay],[Az]])
Ba =  sympy.Matrix([[bax],[bay],[baz]])

Aa = KSa*(A-Ba)

print('a矩阵')
print('$$')
print(sympy.latex(Aa))
print('$$')
print('对Sayx求偏导')
print('$$')
print(sympy.latex(Aa.diff(Sayx)))
print('$$')
print('对Sazx求偏导')
print('$$')
print(sympy.latex(Aa.diff(Sazx)))
print('$$')
print('对Sazy求偏导')
print('$$')
print(sympy.latex(Aa.diff(Sazy)))
print('$$')

print('对Kax求偏导')
print('$$')
print(sympy.latex(Aa.diff(Kax)))
print('$$')
print('对Kay求偏导')
print('$$')
print(sympy.latex(Aa.diff(Kay)))
print('$$')
print('对Kaz求偏导')
print('$$')
print(sympy.latex(Aa.diff(Kaz)))
print('$$')

print('对bax求偏导')
print('$$')
print(sympy.latex(Aa.diff(bax)))
print('$$')
print('对bay求偏导')
print('$$')
print(sympy.latex(Aa.diff(bay)))
print('$$')
print('对baz求偏导')
print('$$')
print(sympy.latex(Aa.diff(baz)))
print('$$')
