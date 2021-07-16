# Dipl

Ovaj repozitorij sadži konačnu inačicu izvornog koda razvijenog za potrebe dipl. rada br. 2213

"Personalizirano daljinskoupravljanje bespilotnom letjelicoms robotskom rukom"

Repozitorij sadrži kod koji omogućuje povezivanje s HRI.ž


Catkin packages used:
  aerial_manipulators_control
  aerial_manipulators_description
  aerial_manipulators_moveit
  dynamixel_workbench_msgs
  gazebo_ros_pkgs
  impedance_control
  mav_comm
  mmuav_gazebo
  orocos_kinematics_dynamics          
  pykdl_utils
  rotors_simulator 
  simple_trajectory
  wit_driver-main
  
Some packages may be installed using rosinstall or apt-get install.

Python version used: Python 2.7.17
Code shoud run on Python 3 as well. Code needs to be run using the same Python version that was used to build ROS packages. 

Instalation: Copy, paste and run

How to run:
1) Run the simulation using: roslaunch mmuav_gazebo uav_attitude_position.launch manipulator_type:="wp_manipulator" start_gazebo:=true z:=1.0 manipulator_tool:="rod"

2) Run the HRI.

3) Run SubPub_UDP_comm.py using ./ or python

HRI MUST BE RUN BEFORE RUNNING SubPub_UDP_comm.py.

This repo also contains modified source code for the HRI. Copy and paste the code in HRI_mapping-data_analysis_multi into Your repository which contains the HRI.






