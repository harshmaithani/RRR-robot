%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RRR robot
% Run this file to view the results of Forward and Inverse Kinematics
% Uncomment the last line to view the whole trajectory 
% OR run complete_planner.m separately

clear all
close all
clc

% Robot DH parameters
% Link | alpha |   a   |   d   |  joint angle
%   0  |   0   |   0   |   -   |     - 
%   1  | pi/2  |  10   |   0   |   theta 1 
%   2  |   0   |   5   |   0   |   theta 2    
%   3  |   0   |   5   |   0   |   theta 3  

L1 = 10;   % Length of Link 1
L2 = 5;    % Length of Link 2
L3 = 5;    % Length of Link 3

% Make the robot using Peter Corke RTB Toolbox
robot = SerialLink( [ Revolute('a',10,'alpha',pi/2,'qlim',[-pi,pi]), Revolute('a',5,'qlim',[-pi/2,pi/2]),Revolute('a',5,'qlim',[-pi,pi])],'name', 'robot')

% 1.Forward Kinematics
J1 =  0.15;   % J1 in rad
J2 =  0.29;   % J2 in rad
J3 =  0.57;   % J3 in rad

q = [J1 J2 J3]; % In rad

[pose,x,y,z,rz,ry,rx] = rtb_fk(robot,q);   % Forward Kinematics using RTB Toolbox
[new_x, new_y, new_z] = forward_kinematics(q);  % Forward Kinematics using analytical equations 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2.Inverse Kinematics

% This is an under actuated manipulator, so we have ignore the rotations of
% the end effector about the x and y axes. Hence we use masking. 

q_inv = robot.ikine(pose,'mask', [1 1 1 0 0 0]); % Inverse Kinematics using RTB Toolbox
new_q = inverse_kinematics(x,y,z); % Inverse Kinematics using analytical equations

%q_inv_deg = [q_inv(1)*180/pi q_inv(2)*180/pi q_inv(3)*180/pi]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3.Validation

disp("Original angles");
disp(q);  

disp("Forward Kinematics using RTB Toolbox");
pose

disp(" ");
disp("Forward Kinematics using analytical equations");
disp([new_x,new_y,new_z]); 

disp("Inverse Kinematics using RTB Toolbox");
disp(q_inv);              

disp("Inverse Kinematics using analytical equations");
disp(new_q);               % 

% Uncomment the below line to display the whole trajectory
% complete_planner;


