function [x,y,z] = forward_kinematics(q)
L1 = 10;
L2 = 5;
L3 = 5;
q1 = q(1);
q2 = q(2);
q3 = q(3);
x = (L1 + L2 * cos(q2) + L3*cos(q2 + q3))*cos(q1);   
y = (L1 + L2 * cos(q2) + L3*cos(q2 + q3))*sin(q1);    
z = L2 * sin(q2) + L3 * sin(q2 + q3);                 
end