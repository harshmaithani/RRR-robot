function [j,a,v,d] = trajectory_planner(qi,qf,movement_time)


    tc = 0.02;                  % cycle time 
    d_travel = qf - qi;         % Distance to be traveled
    
    A = 8*d_travel/(movement_time)^2;
    
    T1 = sqrt(2*d_travel/A);
    T3 = T1;

    t0 = 0;
    t1 = t0+T1;
    t3 = t1+T3;

    m = (2*pi/T1);
    n = 1/m;

    i = 1;
    [j1,a1,v1,d1,k1] = acceleration_phase(t0,t1,tc,i,A,m,n,T1);
    [j2,a2,v2,d2,k2] = deceleration_phase(t1,t3,tc,i,A,m,n,T1,0);

    j = [j1 j2];
    a = [a1 a2];
    v = [v1 v2];
    d = qi+[d1 d2];
    
    if (d_travel==0)
       d = qi;
    end

    function [j,a,v,d,k] = acceleration_phase(t_start,t_end,cycle_time,i,A,m,n,T1)
        for t=t_start:cycle_time:t_end
          j(i) = (A*pi/T1)*sin(m*t);                                  % Jerk
          a(i) = (A/2)*(1-cos(m*t));                                  % Acceleration
          v(i) = (A/2)*(n)*((m*t)-sin(m*t));                          % Velocity 
          d(i) = (A/2)*(n^2)*(0.5*((m*t)^2)-(1-cos(m*t)));            % Displacement
          i=i+1;
        end
        k = i;
    end

    function [j,a,v,d,k] = constant_velocity_phase(t_start,t_end,cycle_time,i,A,m,n,T1)
        for t=t_start:cycle_time:t_end
            j(i) = 0;                                                   % Jerk
            a(i) = 0;                                                   % Acceleration 
            v(i) = 0.5*A*T1;                                            % Velocity
            d(i) = 0.25*A*((T1)^2)+0.5*A*T1*(t-t_start);                % Displacement
          i=i+1;
        end
        k = i;
    end

    function [j,a,v,d,k] = deceleration_phase(t_start,t_end,cycle_time,i,A,m,n,T1,T2)
        for t=t_start:cycle_time:t_end
            j(i) = -(A*pi/T1)*sin(m*(t-t_start));                            % Jerk
            a(i) = -(A/2)*(1-cos(m*(t-t_start)));                            % Acceleration
            v(i) = (A/2)*(n)*( 2*pi*(1-((t-t_start)/T1)) + sin(m*(t-t_start)) );  % Velocity
            d(i) = 0.25*A*(T1^2)+0.5*A*T1*T2+0.5*A*(n^2)*( (((2*pi)^2)*(t-t_start)/T1)-0.5*((m*(t-t_start))^2)+(1-cos(m*(t-t_start))) ) ;                % Displacement  
          i=i+1;
        end
        k = i;
    end

end

