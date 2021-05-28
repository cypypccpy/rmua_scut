#ifndef MOVE_DECISION_DIRECTION_BEHAVIOR_H
#define MOVE_DECISION_DIRECTION_BEHAVIOR_H

namespace  roborts_decision{
class DirectionBehavior{
 public:
    DirectionBehavior(ChassisExecutor* &chassis_executor, 
                                        Blackboard* &blackboard, 
                                        const std::string & proto_file_path) : chassis_executor_(chassis_executor), 
                                                                                                                   blackboard_(blackboard) {
        direction_status = false;
        direction_position_.header.frame_id = "map";
        direction_position_.pose.orientation.x = 0;
        direction_position_.pose.orientation.y = 0;
        direction_position_.pose.orientation.z = 0;
        direction_position_.pose.orientation.w = 1;

        direction_position_.pose.position.x = 0;
        direction_position_.pose.position.y = 0;
        direction_position_.pose.position.z = 0;
        
        if (!LoadParam(proto_file_path)){
            ROS_ERROR("%s can't open file",__FUNCTION__);
        }
    }

    void Run() {
        auto executor_state = Update();
        auto robot_map_pose_ = blackboard_->GetRobotMapPose();
        if (executor_state != BehaviorState::RUNNING) {
            if (direction_status = false && blackboard_->GetBackArmorAttackedStatus() = true) {
                chassis_executor_->Execute(direction_position_);
                direction_status = true;
            }
            else if 
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
        direction_position_.header.frame_id = "map";
        direction_position_.pose.position.x = blackboard_->GetRobotMapPose().x();
        direction_position_.pose.position.y = blackboard_->GetRobotMapPose().y(); 
        direction_position_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                                                                        blackboard_->GetRobotMapPose().roll(), 
                                                                                                                        blackboard_->GetRobotMapPose().pitch(), 
                                                                                                                        blackboard_->GetRobotMapPose().yaw() + 3.14);
        
        return true;
    }

    ~BulletBehavior() = default;
 private:
    roborts_decision::DecisionConfig decision_config;
    ChassisExecutor* const chassis_executor_;
    Blackboard* const blackboard_;
    geometry_msgs::PoseStamped direction_position_;
    bool direction_status;

};
}

#endif