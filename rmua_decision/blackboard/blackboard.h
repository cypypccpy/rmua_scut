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
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "roborts_msgs/ArmorDetectionAction.h"
#include "roborts_msgs/GameZoneArray.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/GameRobotBullet.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/CarCoordinate.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"
#include <thread>

namespace roborts_decision{

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  const std::string &proto_file_path_ = "/home/robotlab/rmua_scut/src/rmua_scut/rmua_decision/config/decision.prototxt";
  explicit Blackboard(const std::string &proto_file_path_):
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true){

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();
    std::cout << costmap_2d_->GetSizeXWorld() << std::endl;
    std::cout << costmap_2d_->GetSizeYWorld() << std::endl;
    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);
    ros::NodeHandle nh;
    game_zone_sub_ = nh.subscribe<roborts_msgs::GameZoneArray>("game_zone_array_status", 1, &Blackboard::GameZoneCallback, this);
    game_time_sub_ = nh.subscribe<roborts_msgs::GameStatus>("game_status", 1 , &Blackboard::GameStatusCallback, this);
    game_bullet_sub_ = nh.subscribe<roborts_msgs::GameRobotBullet>("game_robor_bullet", 1 , &Blackboard::GameBulletCallback, this);
    robot_damage_sub_ = nh.subscribe<roborts_msgs::RobotDamage>("robot_damage", 1 , &Blackboard::RobotDamageCallback, this);
    enemy_pose_sub_ = nh.subscribe<roborts_msgs::CarCoordinate>("carcoordinate", 1 , &Blackboard::EnemyPoseCallback, this);

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path_, &decision_config);

    if (!decision_config.simulate()){

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    }
  }

  ~Blackboard() = default;

  void EnemyPoseCallback(const roborts_msgs::CarCoordinateConstPtr& coord) {
    coord_ = *coord;
    enemy_received_ = true;
  }

  roborts_msgs::CarCoordinate GetEnemyPose() const {
    while(!enemy_received_) {
      ROS_INFO("wait for receive enemy pose");
      ros::spinOnce();
    }
    return coord_;
  }

  void RobotDamageCallback(const roborts_msgs::RobotDamageConstPtr& damage) {
    damage_ = *damage;
    damage_received_ = true;
  }

  roborts_msgs::RobotDamage GetDamage() const {
    while(!damage_received_) {
      ros::spinOnce();
    }
    return damage_;
  }

  void GameBulletCallback(const roborts_msgs::GameRobotBulletConstPtr& bullet) {
    bullet_ = *bullet;
    bullet_received_ = true;
  }

  roborts_msgs::GameRobotBullet GetBullet() const {
    while(!bullet_received_) {
      ros::spinOnce();
    }
    return bullet_;
  }

  void GameStatusCallback(const roborts_msgs::GameStatusConstPtr& status) {
    status_ = *status;
    remaining_time_ = status_.remaining_time;

    if (status_.game_status == 4) {
      begin_ = true;
    }

    time_received_ = true;
  }

  bool GetStatus() const {
    return begin_;
  }

  int GetTime() const {
    ros::Rate rate(1);
    while(!time_received_) {
      ros::spinOnce();
      rate.sleep();
    }
    return remaining_time_;
  }

  void GameZoneCallback(const roborts_msgs::GameZoneArrayConstPtr& zone) {
    
    zone_ = *zone;
    
    float resolution_x = 8680 / 8.20;
    float resolution_y = 5080 / 4.69;
    
    unsigned char blue_bullet = 3;
    unsigned char red_bullet = 4;
    float buff_point[12] = {(530 + 270) / resolution_x, (2850 + 240) / resolution_y, (1930 + 270) / resolution_x, (1710 + 240) / resolution_y,
                              (4070 + 270) / resolution_x, (4095 + 240) / resolution_y, (4070 + 270) / resolution_x, (505 + 240) / resolution_y,
                              (6210 + 270) / resolution_x, (2890 + 240) / resolution_y, (7610 + 270) / resolution_x, (1750 + 240) / resolution_y};
    
    for (int m = 0; m < 6; m++) {
      if (zone_.zone[m].type == blue_bullet) {
        Blackboard::blue_bullet_buff_x = buff_point[0 + m*2];
        Blackboard::blue_bullet_buff_y = buff_point[1 + m*2];
      }

      if (zone_.zone[m].type == red_bullet) {
        Blackboard::red_bullet_buff_x = buff_point[0 + m*2];
        Blackboard::red_bullet_buff_y = buff_point[1 + m*2];
      }
    }
    zone_received_ = true;
  }

  bool GetZone() const {
    while (!zone_received_) {
      ROS_INFO("wait for receive game zone");
      ros::spinOnce();
    }
    return zone_received_;
  }

  float GetZoneX(bool blue_) const {
    if (blue_)
      return blue_bullet_buff_x;
    else
      return red_bullet_buff_x;
  }

  float GetZoneY(bool blue_) const {
    if (blue_)
      return blue_bullet_buff_y;
    else
      return red_bullet_buff_y;
  }

  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;

      double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
          camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);

      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);

      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        if(GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
          enemy_pose_ = global_pose_msg;

        }
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    } else{
      enemy_detected_ = false;
    }

  }

  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }

  bool IsEnemyDetected() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }
  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) const {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

 private:
  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;
  ros::Subscriber game_zone_sub_;
  ros::Subscriber game_time_sub_;
  ros::Subscriber game_bullet_sub_;
  ros::Subscriber robot_damage_sub_;
  ros::Subscriber enemy_pose_sub_;

  float blue_bullet_buff_x, blue_bullet_buff_y, red_bullet_buff_x, red_bullet_buff_y = 0;
  int remaining_time_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;
  bool blue_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  roborts_msgs::GameZoneArray zone_;
  roborts_msgs::GameStatus status_;
  roborts_msgs::GameRobotBullet bullet_;
  roborts_msgs::RobotDamage damage_;
  roborts_msgs::CarCoordinate coord_;

  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;
  bool zone_received_ = false;
  bool time_received_ = false;
  bool bullet_received_ = false;
  bool damage_received_ = false;
  bool enemy_received_ = false;
  bool begin_ = false;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;

};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
