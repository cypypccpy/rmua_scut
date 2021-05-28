#ifndef MOVE_DECISION_BULLET_BEHAVIOR_H
#define MOVE_DECISION_BULLET_BEHAVIOR_H

#include "io/io.h"
#include "line_iterator.h"
#include "../executor/chassis_executor.h"
#include "../proto/decision.pb.h"
#include "../behavior_tree/behavior_state.h"
#include <geometry_msgs/PoseStamped.h>

namespace  roborts_decision{
class BulletBehavior{
 public:
    BulletBehavior(ChassisExecutor* &chassis_executor, 
                                        Blackboard* &blackboard, 
                                        const std::string & proto_file_path) : chassis_executor_(chassis_executor), 
                                                                                                                   blackboard_(blackboard) {
        bullet_status = false;
        bullet_position_.header.frame_id = "map";
        bullet_position_.pose.orientation.x = 0;
        bullet_position_.pose.orientation.y = 0;
        bullet_position_.pose.orientation.z = 0;
        bullet_position_.pose.orientation.w = 1;

        bullet_position_.pose.position.x = 0;
        bullet_position_.pose.position.y = 0;
        bullet_position_.pose.position.z = 0;
        
        if (!LoadParam(proto_file_path)){
            ROS_ERROR("%s can't open file",__FUNCTION__);
        }
    }

    void Run() {
        auto executor_state = Update();
        auto robot_map_pose_ = blackboard_->GetRobotMapPose();
        auto dx = bullet_position_.pose.position.x - robot_map_pose_.pose.position.x;
        auto dy = bullet_position_.pose.position.y - robot_map_pose_.pose.position.y;
        if (executor_state != BehaviorState::RUNNING) {
            if (bullet_status = false) {
                chassis_executor_->Execute(direction_position_);
                blackboard_->GetZone();
            }
        }
        if(std::sqrt(std::pow(dx,2) + std::pow(dy,2)) <= 0.15) {
            direction_status = true;
        }
        if (blackboard_->bullet_ != 0) {
            direction_status = false;
        }
        
    }
    void Cancel() {
        chassis_executor_->Cancel();
        direction_status = false;
    }

    BehaviorState Update() {
        return chassis_executor_->Update();
    }

    bool LoadParam(const std::string &proto_file_path) {
        roborts_decision::DecisionConfig decision_config;
        if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
            return false;
        }
        bullet_position_.header.frame_id = "map";
        if (decsion_config.isblue()) {
            bullet_position_.pose.position.x = blackboard->GetZoneX(bool blue_);
            bullet_position_.pose.position.y = blackboard->GetZoneY(bool blue_);
        }
        else {
            bullet_position_.pose.position.x = blackboard->GetZoneX(bool blue_);
            bullet_position_.pose.position.y = blackboard->GetZoneY(bool blue_);
        }
        return true;
    }

    ~BulletBehavior() = default;
 private:
    roborts_decision::DecisionConfig decision_config;
    ChassisExecutor* const chassis_executor_;
    Blackboard* const blackboard_;
    geometry_msgs::PoseStamped bullet_position_;
    bool bullet_status;

};
}
#endif //MOVE_DECISION_BULLET_BEHAVIOR_H