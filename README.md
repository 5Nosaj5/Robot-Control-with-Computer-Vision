# ME369P-U19
ME 369P Project
The goal of our final project was to develop create a robot simulation that would be able to have its movements controlled using gesture recognition. Below we will explain the different aspects of our code and how each program works.

Training Gesture Recognition Inputs(Final_Project_Hand_Detection.py)
1. Open Final_Project_Hand_Detection.py
2. Make sure that you have mediapipe, cv2, numpy, and pickle installed
3. Make sure you have the ROS Inputs.pkl downloaded (This is the file of trained gestures for movement)
4. Once you run the program you can use the gestures shown in file to output the values right, left, speed 1, speed 2, speed 3, and stop. These will be later used to control the robot in the simulation.

Live Robot Control:
1. Run a virtual machine (VM) or dual boot Ubuntu 22.04 (Virtual Machine Installation: https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)
3. Install ROS2 Humble Hawksbill (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
4. Source ROS2 for every new terminal by default with command: echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
5. Install Gazebo Sim (https://gazebosim.org/docs/garden/install_ubuntu)
6. Install ros_gz (https://github.com/gazebosim/ros_gz)
7. Create a ROS2 package (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
8. Copy the myRobot folder in this repository into the "src" folder of your package
9. If using a VM, enable webcam use by running: "VboxManage controlvm "ROS" webcam attach .1" in a windows terminal while the VM is running. Then in the VM window, go to: "Devices -> Webcams -> Enable HD Webcam". Restart the VM.
10. Create the ROS-GZ bridge with: "ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"
11. In a new terminal, open gz sim with: "gz sim ~/package_name/src/myRobot/myRobot/myModel.sdf"
12. Run the live controller module: "ros2 run myRobot command"
13. Make the trained hand gestures to control the robot. Current commands will be printed to the console
