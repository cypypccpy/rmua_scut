#ifndef ROBORTS_DECISION_SEARCH_BEHAVIOR_H
#define ROBORTS_DECISION_SEARCH_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class SearchBehavior {
 public:
  SearchBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard),
                                                       proto_file_path_(proto_file_path) {


    last_position_.header.frame_id = "map";
    last_position_.pose.orientation.x = 0;
    last_position_.pose.orientation.y = 0;
    last_position_.pose.orientation.z = 0;
    last_position_.pose.orientation.w = 1;

    last_position_.pose.position.x = 0;
    last_position_.pose.position.y = 0;
    last_position_.pose.position.z = 0;

    search_index_ = 0;
    search_count_ = 0;

    blackboard_->GetZone();
    if (!LoadParam(proto_file_path_)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }

  void Run() {

    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      auto robot_map_pose = blackboard_->GetRobotMapPose();

      if (search_count_ == 3) {
        this->SetBeginPosition();

      } else if (search_count_ > 0) {
        auto search_goal = search_region_[(search_index_++ )];
        chassis_executor_->Execute(search_goal);
        search_index_ = (unsigned int) (search_index_% search_region_.size());
        search_count_--;
      } else if (search_count_ == 0) {
        if (blackboard_->GetStatus()) {
          this->IfGetBullet();
          this->IfHide();
        }
        
      }
    }
  }


  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  void SetBeginPosition() {
    if (decision_config.master() && decision_config.blue()) {
      search_region_ = search_region_1_;
      ROS_INFO("search_region_1_");

    } else if (!decision_config.master() && decision_config.blue()) {
      search_region_ = search_region_2_;
      ROS_INFO("search_region_2_");

    } else if (decision_config.master() && !decision_config.blue()) {
      search_region_ = search_region_3_;
      ROS_INFO("search_region_3_");

    } else {
      search_region_ = search_region_4_;
      ROS_INFO("search_region_4_");
    }
    search_count_--;
  }

  void IfGetBullet() {
    time_ = blackboard_->GetTime();
    if (!decision_config.master()) {
      if (time_ < buff_change_time_) {
        search_count_ = 2;
        search_index_ = 0;
        blackboard_->GetZone();
        if (!LoadParam(proto_file_path_)) {
          ROS_ERROR("%s can't open file", __FUNCTION__);
        }
        buff_change_time_ -= 60;
      }
    }
  }
  
  void IfHide() {
    if (decision_config.blue()) {
      roborts_msgs::GameRobotBullet blue_bullet_ = blackboard_->GetBullet();
      if (decision_config.master() && blue_bullet_.blue1 == 0) {
        geometry_msgs::PoseStamped last_position;
        last_position.header.frame_id = "map";
        last_position.pose.position.x = decision_config.blue_master_bot().start_position().x();;
        last_position.pose.position.y = decision_config.blue_master_bot().start_position().y();;
        last_position.pose.position.z = decision_config.blue_master_bot().start_position().z();;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.blue_master_bot().start_position().roll(),
                                                                  decision_config.blue_master_bot().start_position().pitch(),
                                                                  decision_config.blue_master_bot().start_position().yaw());
        last_position.pose.orientation = quaternion;
        
        chassis_executor_->Execute(last_position);
      }
      if (!decision_config.master() && blue_bullet_.blue2 == 0) {
        geometry_msgs::PoseStamped last_position;
        last_position.header.frame_id = "map";
        last_position.pose.position.x = decision_config.blue_wing_bot().start_position().x();;
        last_position.pose.position.y = decision_config.blue_wing_bot().start_position().y();;
        last_position.pose.position.z = decision_config.blue_wing_bot().start_position().z();;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.blue_wing_bot().start_position().roll(),
                                                                  decision_config.blue_wing_bot().start_position().pitch(),
                                                                  decision_config.blue_wing_bot().start_position().yaw());
        last_position.pose.orientation = quaternion;
        
        chassis_executor_->Execute(last_position);
      }
    }

    else {
      roborts_msgs::GameRobotBullet red_bullet_ = blackboard_->GetBullet();
      if (decision_config.master() && red_bullet_.red1 == 0) {
        geometry_msgs::PoseStamped last_position;
        last_position.header.frame_id = "map";
        last_position.pose.position.x = decision_config.red_master_bot().start_position().x();;
        last_position.pose.position.y = decision_config.red_master_bot().start_position().y();;
        last_position.pose.position.z = decision_config.red_master_bot().start_position().z();;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.red_master_bot().start_position().roll(),
                                                                  decision_config.red_master_bot().start_position().pitch(),
                                                                  decision_config.red_master_bot().start_position().yaw());
        last_position.pose.orientation = quaternion;
        
        chassis_executor_->Execute(last_position);
      }
      if (!decision_config.master() && red_bullet_.red2 == 0) {
        geometry_msgs::PoseStamped last_position;
        last_position.header.frame_id = "map";
        last_position.pose.position.x = decision_config.red_wing_bot().start_position().x();;
        last_position.pose.position.y = decision_config.red_wing_bot().start_position().y();;
        last_position.pose.position.z = decision_config.red_wing_bot().start_position().z();;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.red_wing_bot().start_position().roll(),
                                                                  decision_config.red_wing_bot().start_position().pitch(),
                                                                  decision_config.red_wing_bot().start_position().yaw());
        last_position.pose.orientation = quaternion;
        
        chassis_executor_->Execute(last_position);
      }
    }
    
  }

  bool LoadParam(const std::string &proto_file_path) {
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    search_count_ = 3;
    // may have more efficient way to search a region(enemy where disappear)
    search_region_.resize((unsigned int)(4));
    for (int i = 0; i < 1; i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_1(i).x();
      search_point.pose.position.y = decision_config.search_region_1(i).y();
      search_point.pose.position.z = decision_config.search_region_1(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_1(i).roll(),
                                                                decision_config.search_region_1(i).pitch(),
                                                                decision_config.search_region_1(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_1_.push_back(search_point);
    }
    for (int i = 0; i < 2; i++) {
      if (i == 0) {
        geometry_msgs::PoseStamped search_point;
        search_point.header.frame_id = "map";
        search_point.pose.position.x = blackboard_->GetZoneX(decision_config.blue());
        search_point.pose.position.y = blackboard_->GetZoneY(decision_config.blue());
        search_point.pose.position.z = 0.0;
        std::cout << search_point.pose.position.x << std::endl;
        std::cout << search_point.pose.position.y << std::endl;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                                                                  0.0,
                                                                  0.0);
        search_point.pose.orientation = quaternion;
        search_region_2_.push_back(search_point);
        continue;
      }
      else {
        //second point
        geometry_msgs::PoseStamped search_point;
        search_point.header.frame_id = "map";
        search_point.pose.position.x = decision_config.search_region_2(i).x();
        search_point.pose.position.y = decision_config.search_region_2(i).y();
        search_point.pose.position.z = decision_config.search_region_2(i).z();

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_2(i).roll(),
                                                                  decision_config.search_region_2(i).pitch(),
                                                                  1.57 + decision_config.search_region_2(i).yaw());
        search_point.pose.orientation = quaternion;
        search_region_2_.push_back(search_point);
      }
    }

    for (int i = 0; i < 1; i++) {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_3(i).x();
      search_point.pose.position.y = decision_config.search_region_3(i).y();
      search_point.pose.position.z = decision_config.search_region_3(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_3(i).roll(),
                                                                decision_config.search_region_3(i).pitch(),
                                                                decision_config.search_region_3(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_3_.push_back(search_point);
    }

    for (int i = 0; i < 2; i++) {
      if (i == 0) {
        geometry_msgs::PoseStamped search_point;
        search_point.header.frame_id = "map";
        search_point.pose.position.x = blackboard_->GetZoneX(decision_config.blue());
        search_point.pose.position.y = blackboard_->GetZoneY(decision_config.blue());
        search_point.pose.position.z = 0.0;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                                                                  0.0,
                                                                  0.0);
        search_point.pose.orientation = quaternion;
        search_region_4_.push_back(search_point);
        continue;
      }

      else {
        //second point
        geometry_msgs::PoseStamped search_point;
        search_point.header.frame_id = "map";
        search_point.pose.position.x = decision_config.search_region_4(i).x();
        search_point.pose.position.y = decision_config.search_region_4(i).y();
        search_point.pose.position.z = decision_config.search_region_4(i).z();

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_4(i).roll(),
                                                                  decision_config.search_region_4(i).pitch(),
                                                                  1.57 + decision_config.search_region_4(i).yaw());
        search_point.pose.orientation = quaternion;
        search_region_4_.push_back(search_point);
      }
    }
  }
  void SetLastPosition(geometry_msgs::PoseStamped last_position) {
    last_position_ = last_position;
    search_count_ = 3;
  }

  ~SearchBehavior() = default;

 private:
  roborts_decision::DecisionConfig decision_config;
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
  const std::string & proto_file_path_;

  //! chase goal
  geometry_msgs::PoseStamped last_position_;

  //! search buffer
  std::vector<geometry_msgs::PoseStamped> search_region_1_;
  std::vector<geometry_msgs::PoseStamped> search_region_2_;
  std::vector<geometry_msgs::PoseStamped> search_region_3_;
  std::vector<geometry_msgs::PoseStamped> search_region_4_;
  std::vector<geometry_msgs::PoseStamped> search_region_;
  unsigned int search_count_;
  unsigned int search_index_;
  int buff_change_time_ = 120;
  int time_;

};
}

#endif //ROBORTS_DECISION_SEARCH_BEHAVIOR_H
