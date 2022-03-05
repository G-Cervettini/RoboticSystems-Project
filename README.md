# RoboticSystems-Project
Kinematic and dynamic analysis of a 6-axis serial manipulator.

This project uses the Robotics Toolbox library by Peter Corke: https://petercorke.com/toolboxes/robotics-toolbox/

To correctly use the library, extract the file "robot-9.10.zip"
A folder named "rvctool" will be available.
Cut and paste all the repository files inside "rvctools" folder.
Add "rvctools" and its subfolders to Matlab path.

Now you should be able to correctly run the scripts.

"Robot3R_SphericWrist.m" is the main script where the 6-axis manipulator is defined.
This script runs many plots that describe the kinematics and dynamics of the manipulator while it follows a circular path described by "traiettoria_6gdl_v02.mat"
The work envelope of the robot is assessed too.

"SingularityAnalysis.m" is a script devoted to the investigation of the manipulator behavior in some singularity configurations.
The Jacobian matrix is computed to better understand singularity implications.

"cin_inversa.m" is a function that perfoms the inverse kinematics of the 6-axis robot to provide the 8 possible joints configurations.

"Traiettoria_rettilinea_3D.m" Is a script where a straight tool path is planned.
The robot joints configurations needed to follow the path with a given speed law are computed.

For more details on the study and the results, refer to "ReadMe-Project.pdf"
