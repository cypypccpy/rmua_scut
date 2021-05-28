/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <mutex>
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roborts_msgs/TwistAccel.h"
#include "roborts_msgs/GlobalPlannerAction.h"

class VelConverter {
 public:
  VelConverter() : begin_(false), delay(0), mode(0) {
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.angular.z = 0;

    vel_pub_ = cmd_handle_.advertise<roborts_msgs::TwistAccel>("/cmd_vel_acc", 5);
    vel_sub_ = cmd_handle_.subscribe<roborts_msgs::GlobalPlannerActionGoal>("/global_planner_node_action/goal", 5, &VelConverter::goalCB, this);
    result_sub_ = cmd_handle_.subscribe<roborts_msgs::GlobalPlannerActionResult>("/global_planner_node_action/result", 5, &VelConverter::resultCB, this);
    
  }
  void goalCB(const roborts_msgs::GlobalPlannerActionGoal::ConstPtr& goal);
  void resultCB(const roborts_msgs::GlobalPlannerActionResult::ConstPtr& result);
  void UpdateVel();

 private:
  roborts_msgs::TwistAccel cmd_vel_acc_;
  geometry_msgs::Twist cmd_vel_;

  bool new_cmd_acc_, begin_;
  int delay, mode, vel_delay;
  double vel_speed;

  ros::NodeHandle cmd_handle_;
  ros::Publisher vel_pub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber result_sub_;
  std::chrono::high_resolution_clock::time_point time_begin_;

  std::mutex cmd_mutex_;
};

void VelConverter::goalCB(const roborts_msgs::GlobalPlannerActionGoal::ConstPtr& goal)
{
  if (!begin_) {
    begin_ = true;
  }
  ROS_INFO("goal!");
}

void VelConverter::resultCB(const roborts_msgs::GlobalPlannerActionResult::ConstPtr& result)
{
  if (begin_) {
    begin_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ROS_INFO("result!");
}

void VelConverter::UpdateVel() {
  if (begin_) {
    cmd_vel_acc_.twist.linear.x = 0;
    cmd_vel_acc_.twist.linear.y = 0;

    if (mode == 0)
      cmd_vel_acc_.twist.angular.z = 1.2;
    else
      cmd_vel_acc_.twist.angular.z = -1.2;

    cmd_vel_acc_.accel.linear.x = 0;
    cmd_vel_acc_.accel.linear.y = 0;
    cmd_vel_acc_.accel.angular.z = 0;

    vel_pub_.publish(cmd_vel_acc_);
    delay += 1;
    if(delay == 150) {
      mode = mode == 0 ? 1 : 0;
      delay = 0;
    }    
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_converter");

  VelConverter vel_converter;

  while	(ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ros::spinOnce();
    vel_converter.UpdateVel();
  }

  return 0;
}