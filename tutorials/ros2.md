## ROS2 Cheatsheet 
1. Execute `source /opt/ros/foxy/setup.bash` to source the ROS2 workspace 
2. Execute `ros2 run <package_name> <executable_file>` to run a ros2 executable program. 
3. Execute `ros2 launch <package_name> <launch_file>` to run a ros2 node via a launch file.  
### What is a Package?
A package is a folder that contains all the files that make up a particular ROS2 program (packages are the primary organization system for ROS2 programs). A package has the following structure: 
  - `package.xml` is a file that contains meta-information about the package.
  - `setup.py` contians instruction on how to install the package.
  - `setup.cfg` required by ros2 to run any included executables. 
  - `/<package_name>` used by ros2 to find packages. They have to contain `__init__.py` file. 
### Create a Package
