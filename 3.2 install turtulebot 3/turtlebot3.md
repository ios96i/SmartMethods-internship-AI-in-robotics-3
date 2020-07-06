 ## install turtlbot3 ros kinetic
 
 - [ ] install package 
- [ ] install turtlebot simulation package 
 
 
 **open Terminal Ctrl+alt+T**
 
 then write 
 
 `sudo apt-get update` 

` sudo apt-get upgrade` 

` wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh`

---

*The next step is to install dependent packages for TurtleBot3 control*


` sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers` 


` cd ~/catkin_ws/src/` 

` git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git` 

` git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git` 

` cd ~/catkin_ws && catkin_make`  


catkin_make :command is completed without any errors, the preparation for TurtleBot3 is done.

---

Enter the below command on the terminal window of the remote PC to find out the IP address of the remote PC

`ifconfig`

![network_configuration2](https://user-images.githubusercontent.com/62897025/86635495-2505d300-bfa1-11ea-8701-e7105134527d.png)

**Note : save the number the inet addr in your mind**

enter command 

`nano ~/.bashrc`

press alt+/ and rplace the IP with what your computer IP 

![network_configuration3](https://user-images.githubusercontent.com/62897025/86635500-27682d00-bfa1-11ea-967e-1ff8fa4df14a.png)

Press ctrl+X to save 

Then, source the bashrc with below command

`source ~/.bashrc`

- [x] install package  
- [x] install turtlebot simulation package


