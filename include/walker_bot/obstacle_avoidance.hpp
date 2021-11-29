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
 * @file obstacle_avoidance.hpp
 * @author Abhijit Mahalle (abhimah@umd.edu)
 * @brief Header file for the Obstacle Avoidance class
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_WALKER_BOT_OBSTACLE_AVOIDANCE_HPP_ //NOLINT
#define INCLUDE_WALKER_BOT_OBSTACLE_AVOIDANCE_HPP_

#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>

class ObstacleAvoidance {
 public:
  /**
   * @brief Default costructor for the new ObstacleAvoidance object
   * 
   */
  ObstacleAvoidance();

  /**
   * @brief Default destructor for the ObstacleAvoidance object
   * 
   */
  ~ObstacleAvoidance();

  geometry_msgs::Twist velocity(
                    const std::vector<float>& laserscan_data_range,
                    float angle);

 private:
 /**
   * @brief Method that checks if the path is clear of obstacle.
   * 
   * @param laserscan_data_range Obstacle distance
   * @param angle defines the angle that would be beyond the scan range
   * @return true if no obstacles detected
   * @return false if obstacles detected
   */
  bool check_path(const std::vector<float>& laserscan_data_range,
                     float angle);
  geometry_msgs::Twist bot_velocity;

  /**
   * @brief minimum distance of the walker_bot from the obstacle
   * 
   */
  float threshold_dist = 0.2;  // meter

  int turn_count = 0;

  int MAX_ONE_SIDE_TURN_COUNT = 100;
};

#endif  // INCLUDE_WALKER_OBSTACLE_AVOIDANCE_HPP_ //NOLINT
