# requires symbolic package
# pkg install -forge symbolic
# and then 
# pkg load symbolic
# verified with Octave==v5.2 and SymPy=1.5.1 

syms l1 l2 l3 l4
syms j1 j2 j3

Leg
#Insert forward kinematics
fx = -l2 * cos(j2) + l3 * cos(j2+j3) + l1
fy = (l2 * sin(j2) - l3 * sin(j2+j3)) * sin(j1) - l4 * cos(j1)
fz = (-(l2 * sin(j2) - l3 * sin(j2+j3)) * cos(j1) - l4 * sin(j1) )

# Resulting jacobian
xj1 = diff(fx, j1)
xj2 = diff(fx, j2)
xj3 = diff(fx, j3)
yj1 = diff(fy, j1)
yj2 = diff(fy, j2)
yj3 = diff(fy, j3)
zj1 = diff(fz, j1)
zj2 = diff(fz, j2)
zj3 = diff(fz, j3)
J = [ xj1, xj2, xj3 ;
      yj1, yj2, yj3 ;
      zj1, zj2, zj3]
# The jacobian in NOT transposed and NOT inverted