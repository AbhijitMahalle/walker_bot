/**
 * MIT License
 *
 * Copyright (c) 2021 Abhijit Mahalle
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file walker.cpp
 * @author Abhijit Mahalle (abhimah@umd.edu)
 * @brief Executable for the walker class
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <walker_bot/walker.hpp>

Walker:Walker(ros::NodeHandle ros_node_h)
            : obstacle(std::make_shared<ObstacleAvoidance>()) {
  this->ros_node_h = ros_node_h;
  this->velocity_pub = this->ros_node_h.advertise<geometry_msgs::Twist>(
      this->cmdvel_topic, 10);

  this->laser_scan_sub = this->ros_node_h.subscribe(
                                    this->laserscan_topic,
                                    10,
                                    &Walker::laserscan_call_back,
                                    this);
  ros::spinOnce();
}

Walker::~Walker() {
}

void Walker::laserscan_call_back(
    const sensor_msgs::LaserScan::ConstPtr& laserscan_msg) {
  int angle = 10;
  int front_range = 0;
  int left_range = 0 + angle;
  int right_range = 360 - angle;
  geometry_msgs::Twist velocity_cmd;

  ROS_INFO_STREAM("Laser data::"
                  << " Left: " << laserscan_msg->ranges[left_range]
                  << " Front: " << laserscan_msg->ranges[front_range]
                  << " Right: " << laserscan_msg->ranges[right_range]);

  velocity_cmd = this->obstacle_avoider->velocity(
      laserscan_msg->ranges,
      angle);

  this->velocity_pub.publish(velocity_cmd);
}
