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
 * @file walker_bot_node.cpp
 * @author Abhijit Mahalle(abhimah@umd.edu)
 * @brief Walker_bot ROS node
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <cstdlib>

#include <walker_bot/walkerbot_pubsub.hpp>


int main(int argc, char **argv) {
  ros::init(argc, argv, "walker_bot");
  ros::NodeHandle ros_node_h;

  ROS_INFO_STREAM("Walker Node started");
  std::unique_ptr<WalkerBot> obs_avoidance_node(new WalkerBotPubSub(ros_node_h));
  ros::spin();
  return 0;
}
