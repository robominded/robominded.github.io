## ROS2 Cheatsheet 
1. Execute `source /opt/ros/foxy/setup.bash` to source the ROS2 workspace 
2. Execute `ros2 run <package_name> <executable_file>` to run a ros2 executable program. 
3. Execute `ros2 launch <package_name> <launch_file>` to run a ros2 node via a launch file.  
4. Execute `ros2 pkg list` to list all the packages in the current ROS system. Use `ros2 pkg list | grep <token>` to select the packages that contain the `<token>` in their names. 
### What is a Package?
A package is a folder that contains all the files that make up a particular ROS2 program (packages are the primary organization system for ROS2 programs). A package has the following structure: 
  - `package.xml` is a file that contains meta-information about the package.
  - `setup.py` contians instruction on how to install the package.
  - `setup.cfg` required by ros2 to run any included executables. 
  - `/<package_name>` used by ros2 to find packages. They have to contain `__init__.py` file. 
### What is ROS2 Workspace?
It is a directory (a folder) where ROS2 packages reside (usually this directory is called `ros2_ws`. 
### Create and Compile a Package
1- `cd ~/ros2_ws/src`
2- Run `ros2 pkg create --build-type ament_python <package_name> --dependencies <packages>`. `<package_name>` stands for the package that is being created. `<packages>` stands for the packages that the to-be-created package is depending on. 
3- To compile a package first `cd to ~/ros_ws` and then run `colcon build`. Then, run `source install/setup.bash` 


