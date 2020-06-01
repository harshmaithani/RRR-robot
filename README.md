# RRR-robot
An example of computing forward, inverse kinematics, and trajectory planning of a RRR robot in both C++ and MATLAB.  
Refer to **rrr-robot-report.pdf**

# MATLAB

1. Install the toolbox by Peter Corke - RTB-10.4.
2. Run the file **main.m**. This will demonstrate the forward and inverse kinematics.
3. The file **input.mat** contains the desired cartesian trajectory points in the format -   
x &nbsp; y &nbsp; z &nbsp; time  
4. To view the computed joint trajectory run **main.m** with the last line uncommented.

# C++
1. Compile the C++ files using the command  
g++ main.cpp robot.cpp connection.cpp -o main  
2. The file **input.in** contains the desired cartesian trajectory points in the format -   
x &nbsp; y &nbsp; z &nbsp; time  
3. Run the the file **main.exe**
