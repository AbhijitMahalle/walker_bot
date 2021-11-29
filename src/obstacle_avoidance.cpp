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
 * @file obstacle_avoidance.cpp
 * @author Abhijit Mahalle (abhimah@umd.edu)
 * @brief Executable for the ObstacleAvoidance class
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <walker_bot/obstacle_avoidance.hpp>

ObstacleAvoidance::ObstacleAvoidance() {
}

ObstacleAvoidance::~ObstacleAvoidance() {
}

bool ObstacleAvoidance::check_path(
                     const std::vector<float>& laserscan_data_range,
                     float angle) {
  bool clear = true;
  int left = 0 + angle_range;
  int right = 360 - angle_range;

  for (int i=0; i <= angle_range; i++) {
    if (laserscan_data_range[i] <= this->threshold_dist
    || laserscan_data_range[359 - i] <= this->threshold_dist) {
      return false;
    }
  }
  return clear;
}

geometry_msgs::Twist ObstacleAvoidance::velocity(
                       const std::vector<float>& laserscan_data_range,
                        float angle) {
  if (this->check_path(laserscan_data_range, angle)) {
    this->bot_velocity.linear.x = 0.7;
    this->bot_velocity.angular.z = 0.0;
  } else {
    ROS_WARN_STREAM("Obstacle detected");
    this->bot_velocity.linear.x = 0.0;
    this->bot_velocity.angular.z =
                (this->turn_count < MAX_ONE_SIDE_TURN_COUNT) ? 1.0 : -1.0;
    this->turn_count = (this->turn_count + 1) % (MAX_ONE_SIDE_TURN_COUNT * 2);
  }
  return bot_velocity;
}
