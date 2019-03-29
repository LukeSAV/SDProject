syms phin an Rn xn yn

phi = phin+an;
R = Rn;
a = an;
x = xn + Rn*sin(phin)-Rn*sin(phin)*cos(an) - Rn*cos(phin)*sin(an);
y = yn - Rn*cos(phin)-Rn*sin(phin)*sin(an) + Rn*cos(phin)*cos(an);

J = jacobian([phi; a; R; x; y], [phin an Rn xn yn]);

last_state = [0.3927; 0.3927; 13.5000; -5.1662; -1.0276];
J=subs(J, {phin, an, Rn, xn, yn}, {0.3927, 0.3927, 13.5000, -5.1662, -1.0276});
J=vpa(J)
x=subs(x, {phin, an, Rn, xn, yn}, {0.3927, 0.3927, 13.5000, -5.1662, -1.0276});
y=subs(y, {phin, an, Rn, xn, yn}, {0.3927, 0.3927, 13.5000, -5.1662, -1.0276});
x=vpa(x)
y=vpa(y)

state = J*last_state




