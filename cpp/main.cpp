/* Remy Robotics Test

DH Parameters
Link  alpha    a      d     joint angle
  0     0      0      -        - 
  1   pi/2     10     0      q1 
  2     0      5      0      q2    
  3     0      5      0      q3   
  
*/

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

int main()
{
    Robot robot;
    Connection connection;
    
    // Sample for testing Forward and Inverse Kinematics
    // ------
    float q1_test = 0.15;     // Enter in radians
    float q2_test = 0.29;     // Enter in radians
    float q3_test = 0.57;     // Enter in radians
    
    robot.forward_kinematics(q1_test,q2_test,q3_test);   // Angles are in radians
    robot.print_forward_kinematics();

    robot.inverse_kinematics(robot.x,robot.y,robot.z);
    robot.print_inverse_kinematics();
    // ------
    
    // Sample for testing Connection class
    float angles_send[3];                           // Joint angles in radians to send data
    float q1_received, q2_received, q3_received;    // Joint angles in radians to receive data
    std::vector<unsigned char> angleVector;         // Vector to send data to robot
    
    angles_send[0] = 1.57;
    angles_send[1] = 3.14;
    angles_send[2] = 0.78;

    connection.send(angleVector, angles_send[0], angles_send[1], angles_send[2]);
    connection.receive(angleVector, q1_received, q2_received, q3_received);
    
    std::cout<<"----"<<std::endl;
    std::cout<<"angle 0: "<<angles_send[0]<<std::endl;
    std::cout<<"angle 1: "<<angles_send[1]<<std::endl;
    std::cout<<"angle 2: "<<angles_send[2]<<std::endl;
    std::cout<<"----"<<std::endl;
    
    std::cout << "q1: " << q1_received << std::endl; // Print q1
    std::cout << "q2: " << q2_received << std::endl; // Print q2
    std::cout << "q3: " << q3_received << std::endl; // Print q3
    std::cout<<"----"<<std::endl;
    
    std::vector<std::vector<float>> trajectory_points;                  // Vector for storing data points from file 
    
    std::ifstream fin("input.in");                                       // Read the file 
    for (std::string line; std::getline(fin, line); ){
            std::istringstream in(line);
            trajectory_points.push_back(std::vector<float>(std::istream_iterator<float>(in),std::istream_iterator<float>()));
        }
    
    std::cout<<"-----Data points---"<<std::endl;                                   // Put data points in a 2D vector
    for (int i = 0; i < trajectory_points.size(); i++) { 
        for (int j = 0; j < trajectory_points[i].size(); j++){ 
            std::cout << trajectory_points[i][j] << " ";
        }
        std::cout << std::endl; 
    } 
    
    std::vector <std::vector<float>> trajectory_total;
    trajectory_total = robot.complete_planner(trajectory_points);               // Trajectory planner
    
    return 0;
}

