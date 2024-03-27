# ur_manipulation_repo

After creating workspace, clone this repository into your src directory of the workspace.

motion_generator_library is the main package handling motions in joint space and cartesian space.

1) For obtaining movements of UR5 joints as a function of sine, please run - 
   roslaunch motion_generator_library task1.launch

2) Motion library providing joint space and cartesian space is available at motion_generator_library/src/motion_generator_library.cpp
   For cartesian space motion, two approaches are developed i) MoveIt 2) With Orocos KDL
   i) MoveIt! - In the first terminal, start gazebo node with - roslaunch ur_gazebo ur5_bringup.launch
                In the second termina, run - roslaunch motion_generator_library task2.launch

   ii) orocos kdl - To run this, launch the gazebo in the first terminal and in the second terminal, run - roslaunch ur_move ur_cartesian_motion_node
                     (This approach is not working , hence MoveIt is used for creating motion library)

3) A python API based on the motion library is created. For establishing communication between C++ library and the python API, a custom service is created.
   Service is advertised with a motion_library_wrapper_node written in C++ and the service client is written as a python script, acting as API.

   In the first terminal, start gazebo node with - roslaunch ur_gazebo ur5_bringup

   In the second terminal, start the MoveIt and service server with - roslaunch motion_generator_library task3.launch

   In the third terminal, run the python API with - rosrun motion_generator_library robot_motion_api.py
        
