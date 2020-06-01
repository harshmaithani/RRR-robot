#ifndef ROBOT_H
#define ROBOT_H

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <fstream>

#include <iterator>
#include <sstream>

#define PI 3.14159265

class Robot{
    private: 
        float L1 = 10;  
        float L2 = 5;  
        float L3 = 5;   
        
        float q1_min = -PI, q1_max = PI;       // Minimum and maximum limits of Joint 1
        float q2_min = -PI/2, q2_max = PI/2;   // Minimum and maximum limits of Joint 2
        float q3_min = -PI, q3_max = PI;       // Minimum and maximum limits of Joint 3
        
        float m,n;        // For internal calculation of inverse kinematics 
        
    public: 
        Robot(){}
        ~Robot(){}
    
        float q1;         // Joint angle 1 (radians)
        float q2;         // Joint angle 2 (radians)
        float q3;         // Joint angle 3 (radians)
        
        float x;          // x position 
        float y;          // y position 
        float z;          // z position 
     
        void forward_kinematics(float, float, float);           // Forward Kinematics function
        void inverse_kinematics(float, float, float);           // Inverse Kinematics function
        void print_forward_kinematics(void);                    // Print robot details
        void print_inverse_kinematics(void);                    // Print robot details 
        
        float q1_limit_check(float);  // Check limits of q1 
        float q2_limit_check(float);  // Check limits of q2 
        float q3_limit_check(float);  // Check limits of q3
        
        std::vector<float> acceleration_phase(float,float,float,int,float,float,float);               // Acceleration phase function
        std::vector<float> deceleration_phase(float,float,float,int,float,float,float,float,float); // Deceleration phase function
        std::vector<float> trajectory_planner(float,float,float);                                        // Point to point Trajectory planner
        std::vector <std::vector<float>> complete_planner(std::vector<std::vector<float>>);               // Complete trajectory planner
};

#endif
