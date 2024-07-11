
syms t p0 p1 p2 p3 real

b = (1-t)^3*p0 + 3*(1-t)^2*t*p1 + 3*(1-t)*t^2*p2 + t^3*p3

b_ = simplify(diff(b, t))
b__ = simplify(diff(b_,t))
