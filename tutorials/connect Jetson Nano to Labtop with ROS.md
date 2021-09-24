# Connect Jetson Nano (or any computer) with a Labtop on the same network

- Open terminal in the first computer (e.g., Jetson Nano or Raspberry PI) and execute the following
   - `export ROS_MASTER_URI=http://Labtop IP address:11311` [Assuming you want ROS Master to run on the labtop]
   - `export ROS_HOSTNAME=first computer IP address` (e.g., Jetson Nano IP address)
  
- To make these changes permanent add them to your .bashrc script as follows
   - `echo "export ROS_MASTER_URI=http://Labtop IP address:11311" >> ~/.bashrc`
   - `echo "ROS_HOSTNAME=first computer IP address" >> ~/.bashrc`
 
