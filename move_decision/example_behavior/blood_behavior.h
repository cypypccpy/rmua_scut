#ifndef MOVE_DECISION_BLOOD_BEHAVIOR_H
#define MOVE_DECISION_BLOOD_BEHAVIOR_H

#include "io/io.h"
#include "line_iterator.h"
#include "../executor/chassis_executor.h"
#include "../proto/decision.pb.h"
#include "../behavior_tree/behavior_state.h"
#include <geometry_msgs/PoseStamped.h>

namespace roborts_decision{
class BloodBehavior{
  private:
    BloodBehavior(ChassisExecutor* &chassis_executor, 
                    Blackboard* &blackboard, 
                    onst std::string & proto_file_path) : chassis_executor_(chassis_executor), 
                                                        blackboard_(blackboard) {
    
        blood_status = false;
        blood_position_.header.frame_id = "map";
        blood_position_.pose.orientation.x = 0;
        blood_position_.pose.orientation.y = 0;
        blood_position_.pose.orientation.z = 0;
        blood_position_.pose.orientation.w = 1;

        blood_position_.pose.position.x = 0;
        blood_position_.pose.position.y = 0;
        blood_position_.pose.position.z = 0;
        
        if (!LoadParam(proto_file_path)){
            ROS_ERROR("%s can't open file",__FUNCTION__);
        }    
    }

    void Run() {
        auto executor_state = Update();
        auto robot_map_pose_ = blackboard_->GetRobotMapPose();
        auto dx = blood_position_.pose.position.x - robot_map_pose_.pose.position.x;
        auto dy = blood_position_.pose.position.y - robot_map_pose_.pose.position.y;

        if (executor_state != BehaviorState::RUNNING) {
            if (blood_status = false) {
                chassis_executor_->Execute(blood_position_);
                blackboard_->GetHPZone();
            }
        }
        if(std::sqrt(std::pow(dx,2) + std::pow(dy,2)) <= 0.15) {
            blood_status = true;
        }
        if (blackboard_->hp_ >= 200) {
            blood_status = false;
    }

    void Cancel() {
        chassis_executor_->Cancel();
        blood_status = false;
    }

    BehaviorState Update() {
        return chassis_executor_->Update();
    }

    bool LoadParam(const std::string &proto_file_path) {
        roborts_decision::DecisionConfig decision_config;
        if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
            return false;
        }
        else{           
            blood_position_.header.frame_id = "map";
            blood_position_.pose.position.x = blackboard->GetHPZoneX();
            blood_position_.pose.position.y = blackboard->GetHPZoneY();
            return true;
        }
        
    }

    ~BloodBehavior() = default;

  private:

    roborts_decision::DecisionConfig decision_config;
    ChassisExecutor* const chassis_executor_;
    Blackboard* const blackboard_;
    geometry_msgs::PoseStamped blood_position_;
    bool blood_status;
        
};
}
#endif