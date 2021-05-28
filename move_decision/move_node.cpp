#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"

#include "example_behavior/line_iterator.h"

#include "example_behavior/frozen_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/bullet_behavior.h"
#include "example_behavior/blood_behavior.h"
#include "example_behavior/direction_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/escape_behavior.h"

enum BehaviorStateEnum{
    INIT =  -1,
    FROZEN = 0,
    SEARCH = 1,
    BULLET = 2,
    BLOOD = 3,
    DIRECTION = 4,
    GOAL = 5,
    ESCAPE = 6,
};

int main(int argc, char **argv){
    ros::init(argc, argv, "move_node");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
    auto chassis_executor = new roborts_decision::ChassisExecutor;
    auto gimbal_executor = new roborts_decision::GimbalExecutor;
    auto blackboard = new roborts_decision::Blackboard(full_path);
    
    BehaviorStateEnum last_state, cur_state;
    last_state = BehaviorStateEnum::INIT;
    cur_state = BehaviorStateEnum::INIT;

    roborts_decision::FrozenBehavior frozen_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::SearchBehavior search_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::BulletBehavior bullet_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::BloodBehavior blood_behavior(chassis_executor, blacboard, full_path);
    roborts_decision::DirectionBehavior direction_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::GoalBehavior goal_behavior(chassis_executor, blackboard, full_path);
    roborts_decision::EscapeBehavior escape_behavior(chassis_executor,blackboard, full_path);
    ros::Rate rate(10);


    while(ros::ok()){
        ros::spinOnce();

        if (blackboard_-> begin_ = false){
            printf("Game hasn't begun. Waiting for referee system msgs...........\n");
            cur_state = BehaviorStateEnum::FROZEN;
        }
        else if (blackboard_->begin_ = true){
            if(blackboard->bullet_ == 0){
                cur_state = BehaviorStateEnum::BULLET;
            }
            else if(blackboard_->hp_ <= 500){
                cur_state = BehaviorStateEnum::BLOOD;
            }
            else if (blackboard_-> hp_ >=1000 && blackboard_->backattacked_= true ){
                cur_state = BehaviorStateEnum::DIRECTION;
            }
            else if (blackboard_-> hp_ <=1000 && blackboard_->backattacked_= true ){
                cur_state = BehaviorStateEnum::ESCAPE;
            }
            else {
                cur_state = BehaviorStateEnum::SEARCH;
            }

        if (last_state != BehaviorStateEnum::INIT && last_state != cur_state){
            switch (cur_state){
                case BehaviorStateEnum::FROZEN:
                    frozen_behavior.Cancel();
                    std::cout<<"stop chassis frozen"<<std::endl;
                    break;
                case BehaviorStateEnum::BULLET:
                    bullet_behavior.Cancel();
                    std::cout<<"stop getting bullet"<<std::endl;
                    break;
                case BehaviorStateEnum::BLOOD:
                    blood_behavior.Cancel();
                    std::cout<<"stop getting blood"<<std::endl;
                    break;
                case BehaviorStateEnum::DIRECTION:
                    direction_behavior.Cancel();
                    std::cout<<"stop direction change"<<std::endl;
                    break;
                case BehaviorStateEnum::ESCAPE:
                    escape_behavior.Cancel();
                    std::cout<<"stop escaping"<<std::endl;
                    break;
                case BehaviorStateEnum::SEARCH:
                    search_behavior.Cancel();
                    std::cout<<"stop searching"<<std::endl;
            }
        }
        switch (cur_state){
            case BehaviorStateEnum::FROZEN:
                frozen_behavior.Run();
                std::cout<<"chassis frozen"<<std::endl;
                break;
            case BehaviorStateEnum::BULLET:
                bullet_behavior.Run();
                std::cout<<"getting bullet"<<std::endl;
                break;
            case BehaviorStateEnum::BLOOD:
                blood_behavior.Run();
                std::cout<<"getting blood"<<std::endl;
                break;
            case BehaviorStateEnum::DIRECTION:
                direction_behavior.Run();
                std::cout<<"direction change"<<std::endl;
                break;
            case BehaviorStateEnum::ESCAPE:
                escape_behavior.Run();
                std::cout<<"escape from battle"<<std::endl;
                break;
            case BehaviorStateEnum::SEARCH:
                search_behavior.Run();
                std::cout<<"begin to search"<<std::endl;
            }

            last_state = cur_state;
            rate.sleep();
        }
    }
    return 0;
}
