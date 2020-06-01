function [pose,x,y,z,rz,ry,rx] = rtb_fk(robot,q)
q_rad = [q(1) q(2) q(3)];
pose = robot.fkine(q_rad);
pose2_rcv(:,1) = pose.n;
pose2_rcv(:,2) = pose.o;
pose2_rcv(:,3) = pose.a;

pose_rpy_deg = rotm2eul(pose2_rcv)*180/pi;

x = pose.t(1);        
y = pose.t(2);        
z = pose.t(3);        

rz = pose_rpy_deg(1);      % deg
ry = pose_rpy_deg(2);      % deg
rx = pose_rpy_deg(3);      % deg

end