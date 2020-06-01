#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <fstream>

#include <iterator>
#include <sstream>

#include "robot.h"
#include "connection.h"

#define PI 3.14159265

void Robot::forward_kinematics(float q1_in,float q2_in,float q3_in){
    q1 = q1_in;   // q1 is in radians
    q2 = q2_in;   // q2 is in radians
    q3 = q3_in;   // q3 is in radians 
    
    x = (L1 + L2 * cos(q2) + L3*cos(q2 + q3))*cos(q1);   
    y = (L1 + L2 * cos(q2) + L3*cos(q2 + q3))*sin(q1);   
    z = L2 * sin(q2) + L3 * sin(q2 + q3);               
}

void Robot::inverse_kinematics(float x_in, float y_in, float z_in){
    x = x_in;  
    y = y_in;  
    z = z_in;  
    
    q1 = atan(y/x);    // In radians 
    q3 = acos((pow((x/cos(q1) - L1),2) + pow(z,2) - pow(L2,2) - pow(L3,2))/(2*L2*L3));  // In radians 
    
    m = L2 + L3*cos(q3);
    n = L3 * sin(q3);
    q2 = (atan2(m,n)-atan2(sqrt(pow(n,2)+pow(m,2)-pow(z,2)),z));  // In radians
}

void Robot::print_forward_kinematics(){
    std::cout<<"Forward Kinematics ------"<<std::endl;
    std::cout<<"q1: "<<q1<<"\t q2: "<<q2<<"\t q3: "<<q3<<std::endl;  
    std::cout<<"x: "<<x<<"\t y: "<<y<<"\t z: "<<z<<std::endl;
}

void Robot::print_inverse_kinematics(){
    std::cout<<"Inverse Kinematics ------"<<std::endl;
    std::cout<<"x: "<<x<<"\t y: "<<y<<"\t z: "<<z<<std::endl;
    std::cout<<"q1: "<<q1<<"\t q2: "<<q2<<"\t q3: "<<q3<<std::endl;  
}

float Robot::q1_limit_check(float q1_in){   // Check limits of q1
    float q1_corrected;
    if ((q1_in >= q1_min) && (q1_in <= q1_max)){
        q1_corrected = q1_in;
    }
    if (q1_in < q1_min){
        q1_corrected = q1_min;
    }
    if (q1_in > q1_max){
        q1_corrected = q1_max;
    }
    return q1_corrected;
}  

float Robot::q2_limit_check(float q2_in){   // Check limits of q2
    float q2_corrected;
    if ((q2_in >= q2_min) && (q2_in <= q2_max)){
        q2_corrected = q2_in;
    }
    if (q2_in < q2_min){
        q2_corrected = q2_min;
    }
    if (q2_in > q2_max){
        q2_corrected = q2_max;
    }
    return q2_corrected;
}  

float Robot::q3_limit_check(float q3_in){   // Check limits of q3
    float q3_corrected;
    if ((q3_in >= q3_min) && (q3_in <= q3_max)){
        q3_corrected = q3_in;
    }
    if (q3_in < q3_min){
        q3_corrected = q3_min;
    }
    if (q3_in > q3_max){
        q3_corrected = q3_max;
    }
    return q3_corrected;
}  

std::vector<float> Robot::acceleration_phase(float t_start,float t_end,float cycle_time,int i,float A,float m,float n){
        int k;
        std::vector<float> d;
        float displacement;
        for (float t=t_start;t<t_end;t=t+cycle_time){
            displacement = (A/2)*(pow(n,2))*(0.5*(pow((m*t),2))-(1-cos(m*t)));  // Displacement
            d.push_back(displacement);    
            i=i+1;
        }
        k = i;
        return d;
}

std::vector<float> Robot::deceleration_phase(float t_start,float t_end,float cycle_time,int i,float A,float m,float n,float T1,float T2){
        int k;
        std::vector<float> d;
        float displacement;
        for (float t=t_start;t<t_end;t=t+cycle_time){
            displacement = 0.25*A*(pow(T1,2))+0.5*A*T1*T2+0.5*A*(pow(n,2))*( (pow((2*PI),2)*(t-t_start)/T1)-0.5*pow((m*(t-t_start)),2)+(1-cos(m*(t-t_start))) ) ;   // Displacement  
            d.push_back(displacement);    
            i=i+1;
        }
        k = i;
        return d;
}

std::vector<float> Robot::trajectory_planner(float qi,float qf,float movement_time){
        float tc = 0.02;                  // cycle time 
        float d_travel = qf - qi;         // Distance to be traveled
     
        float A = 8*d_travel/(pow((movement_time),2));
    
        float T1 = sqrt(2*d_travel/A);
        float T2 = T1;

        float t0 = 0;
        float t1 = t0+T1;
        float t2 = t1+T2;

        float m = (2*PI/T1);
        float n = 1/m;

        int i = 1;
        
        std::vector<float> d_acc = acceleration_phase(t0,t1,tc,i,A,m,n);
        std::vector<float> d_dec = deceleration_phase(t1,t2,tc,i,A,m,n,T1,0);
        d_acc.insert(d_acc.end(), d_dec.begin(), d_dec.end() );
        for (int i = 0; i < d_acc.size(); i++) { 
            d_acc[i] = d_acc[i] + qi;
        }
        
        if (d_travel ==0){
            d_acc.push_back(qi);
        }
        
        return d_acc;
}

std::vector <std::vector<float>> Robot::complete_planner(std::vector<std::vector<float>> values){
    float x1,y1,z1,t1;                              // Start points
    float x2,y2,z2,t2;                              // Target points
    
    float q1_received, q2_received, q3_received;    // Joint angles in radians to receive data
    std::vector<unsigned char> angleVector;         // Vector to send data to robot
    
    float q1_corrected, q2_corrected, q3_corrected; // Check for joint limits
    
    Robot robot_initial;
    Robot robot_final;
    Robot robot_planner;
    Connection connection_robot;
    
    std::vector <std::vector<float>> trajectory_complete; // Vector to store complete trajectory 
    
    for (int i = 0; i < values.size()-1; i++) { 
        std::cout<<"Trajectory: "<<i+1<<std::endl;
        
        x1 = values[i][0];    
        y1 = values[i][1];    
        z1 = values[i][2];    
        t1 = values[i][3]; 
        
        std::cout<<"i: "<<i+1<<"\t x1: "<<x1<<"\t y1: "<<y1<<"\t z1: "<<z1<<"\t t1: "<<t1<<std::endl;
        
        x2 = values[i+1][0];   
        y2 = values[i+1][1];  
        z2 = values[i+1][2];   
        t2 = values[i+1][3];
        
        std::cout<<"i: "<<i+1<<"\t x2: "<<x2<<"\t y2: "<<y2<<"\t z2: "<<z2<<"\t t2: "<<t2<<std::endl;
       
        robot_initial.inverse_kinematics(x1,y1,z1);
        robot_final.inverse_kinematics(x2,y2,z2);
        
        std::vector<float> trajectory_q1,trajectory_q2,trajectory_q3;  // Vectors to store trajectory of single joint movement 
        robot_initial.print_inverse_kinematics();
        robot_final.print_inverse_kinematics();

        trajectory_q1 = robot_planner.trajectory_planner(robot_initial.q1 ,robot_final.q1 ,t2-t1);  // Trajectory of q1 
        trajectory_q2 = robot_planner.trajectory_planner(robot_initial.q2 ,robot_final.q2 ,t2-t1);  // Trajectory of q2 
        trajectory_q3 = robot_planner.trajectory_planner(robot_initial.q3 ,robot_final.q3 ,t2-t1);  // Trajectory of q3
        
  
        int vector_size;
        if (trajectory_q1.size()>1){                           // Determine the maximum size
            vector_size = trajectory_q1.size();
        }
        else if (trajectory_q2.size()>1){
            vector_size = trajectory_q2.size();    
        }
        else if (trajectory_q3.size()>1){
            vector_size = trajectory_q3.size();
        }
       // std::cout<<"Vector size is "<<vector_size<<std::endl;
        
       for (int i=0;i<vector_size;i++){        // Printing entire vector
            std::vector <float> trajectory_current;  // Vector to store trajectory of all joints between two data points from file
                if (trajectory_q1.size()>1){
                    q1_corrected = robot_planner.q1_limit_check(trajectory_q1[i]);  // Check if q1 cross joint limits
                    trajectory_current.push_back(q1_corrected);
                    //trajectory_current.push_back(trajectory_q1[i]);
                }
                else{
                    trajectory_current.push_back(robot_initial.q1);
                }
                
                if (trajectory_q2.size()>1){ 
                    q2_corrected = robot_planner.q2_limit_check(trajectory_q2[i]);   // Check if q2 crosses joint limits
                    trajectory_current.push_back(q2_corrected);
                    //trajectory_current.push_back(trajectory_q2[i]);
                }
                else{
                    trajectory_current.push_back(robot_initial.q2);
                }
                
                if (trajectory_q3.size()>1){    
                    q3_corrected = robot_planner.q3_limit_check(trajectory_q3[i]);   // Check if q3 crosses joint limits
                    trajectory_current.push_back(q3_corrected);
                    //trajectory_current.push_back(trajectory_q3[i]);
                }
                else{
                    trajectory_current.push_back(robot_initial.q3);
                }
                
                connection_robot.send(angleVector, trajectory_current[0], trajectory_current[1], trajectory_current[2]);  // Send values to robot via data vector
                connection_robot.receive(angleVector, q1_received, q2_received, q3_received);   // Receive joint values from robot
    
                trajectory_complete.push_back(trajectory_current);
                
            std::cout<<q1_received<<" "<<q2_received<<" "<<q3_received<<std::endl;    
      
        }
    }    
    return trajectory_complete;    
}


