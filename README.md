# Walker Bot

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
 ---
## Overview
Walker bot is a ROS package based on turtlebot3 capable of avoiding obstacles in an environment. It uses laserscan data to detect obstacles. 

## Dependencies
- Ubuntu 18.04 (LTS)
- ROS Melodic
- Turtlebot3 ROS 

## Steps to install
### Turtlebot3 installation:
`sudo apt-get install ros-melodic-turtlebot3` \
`sudo apt-get install ros-melodic-turtlebot3-simulations` \
`echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc` 
   
### To install the walker_bot package, enter following commands in the 'src/' directory of your catkin workspace
`git clone https://github.com/abhi-mah/walker_bot.git` \
`cd ..` \
`catkin_make --pkg walker_bot` \
`source devel/setup.bash` 

### Launch walker_bot
In your catkin workspace, enter following command to launch walker_bot_node. \
`roslaunch walker_bot walker_bot_node.launch`

### Launch walker_bot with rosbag record
`roslaunch walker_bot walker_bot_node.launch record_bag:=true`

Pre-recorded rosbag can be found [here](https://drive.google.com/file/d/109kXrhj6j0ESQNsCP5jkx7czrwJffC46/view?usp=sharing).

### Inspecting generated bag files
Use following command to inspect the generated bag file, \
`roscd walker_bot/results/` \
`rosbag info my_bag.bag`

To play the rosbag run following command. \
`rosbag play my_bag.bag`



    
