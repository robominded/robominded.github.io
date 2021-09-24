# Connect two computers with ROS
### Connect two computers (e.g., Jetson Nano with a labtop) running on the same network

- Open terminal in computer_1 (e.g., Jetson Nano or Raspberry PI) and execute the following
   - `export ROS_MASTER_URI=http://computer_2 IP address:11311 #Assuming ROS Master will run on the computer_2`
   - `export ROS_HOSTNAME=computer_1 IP address` 
- Open terminal in computer_2 and execute the following
   - `export ROS_MASTER_URI=http://computer_2 IP address:11311` 
   - `export ROS_HOSTNAME=computer_2 IP address` 
  
*To make these changes permanent add them to the.bashrc file as follows*
- Open terminal in computer_1 and run
   - `echo "export ROS_MASTER_URI=http://computer_2:11311" >> ~/.bashrc`
   - `echo "ROS_HOSTNAME=computer_1" >> ~/.bashrc`
- Open terminal in computer_2 and run
   - `echo "export ROS_MASTER_URI=http://computer_2:11311" >> ~/.bashrc`
   - `echo "ROS_HOSTNAME=computer_2" >> ~/.bashrc`
 
