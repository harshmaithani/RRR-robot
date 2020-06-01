% Run this file to generate the complete trajectory

data = load('input.mat');         % Load the data file
trajectory_points = data.data

trajectory_total = [];  

for i=1:size(trajectory_points,1)-1
    x1 = trajectory_points(i,1);  % Start points
    y1 = trajectory_points(i,2);
    z1 = trajectory_points(i,3);
    t1 = trajectory_points(i,4);
    
    q_initial = inverse_kinematics(x1,y1,z1);
    
    x2 = trajectory_points(i+1,1); % Goal
    y2 = trajectory_points(i+1,2);
    z2 = trajectory_points(i+1,3);
    t2 = trajectory_points(i+1,4);
    
    q_final = inverse_kinematics(x2,y2,z2);
    
    [j,a,v,q1_d] = trajectory_planner(q_initial(1),q_final(1),t2-t1);   % Trajectory planning of q1
    [j,a,v,q2_d] = trajectory_planner(q_initial(2),q_final(2),t2-t1);   % Trajectory planning of q2
    [j,a,v,q3_d] = trajectory_planner(q_initial(3),q_final(3),t2-t1);   % Trajectory planning of q3
    
    q1_d = q1_d';
    q2_d = q2_d';
    q3_d = q3_d';
    
    q1_size = size(q1_d,1);
    q2_size = size(q2_d,1);
    q3_size = size(q3_d,1);

    q_size_max = max([q1_size q2_size q3_size]);
    
    if (q1_size ~= q_size_max)
        q1_d = q_initial(1) * ones(q_size_max,1);
    end
    if (q2_size ~= q_size_max)
        q2_d = q_initial(2) * ones(q_size_max,1);
    end
    if (q3_size ~= q_size_max)
        q3_d = q_initial(3) * ones(q_size_max,1);
    end
    
    disp("Trajectory");
    print1 = ['i:',num2str(i),'      x1: ',num2str(x1),'      y1: ',num2str(y1),'      z1: ',num2str(z1), '      t1: ',num2str(t1)];
    print2 = ['i:',num2str(i),'      x2: ',num2str(x2),'      y2: ',num2str(y2),'      z2: ',num2str(z2), '      t2: ',num2str(t2)];
    disp(print1);
    disp(print2);
    
    trajectory_current = [q1_d q2_d q3_d]        % Trajectory of current trial
    trajectory_total = [trajectory_total;trajectory_current];   % Append to the total trajectory
        
end


