function [q] = inverse_kinematics(x,y,z)

L1 = 10;
L2 = 5;
L3 = 5;

theta1 = atan(y/x);
theta3 = acos(((x*sec(theta1) - L1)^2 + z^2 - L2^2 - L3^2 )/(2*L2*L3));

m = L2+L3*cos(theta3);
n = L3*sin(theta3);

a = n;
b = m;
c = z;

theta2 = atan2(b,a)-atan2(sqrt(a^2+b^2-c^2),c);

q = [theta1 theta2 theta3];
end