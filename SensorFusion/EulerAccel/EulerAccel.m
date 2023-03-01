function[phi, theta] = EulerAccel(ax, ay, az)
%
%
g = 9.81;

theta = asin( ax / g);
phi = asin( -ay / (g*cos(theta)) );