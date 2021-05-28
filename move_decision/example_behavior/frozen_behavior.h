#ifndef MOVE_DECISION_FROZEN_BEHAVIOR_H
#define MOVE_DECISION_FROZEN_BEHAVIOR_H

#include "io/io.h"
#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "line_iterator.h"
namespace roborts_decision{
class FrozenBehavior{
    public:
        FrozenBehavior(ChassisExecutor* &chassis_executor, 
                                        Blackboard* &blackboard, 
                                        const std::string & proto_file_path) : chassis_executor_(chassis_executor), 
                                                                                                                   blackboard_(blackboard) {
            frozen_status = false;
        }

        void Run() {
            auto executor_state = Update();
            if (executor_state !=  BehaviorState::RUNNING){
                chassis_executor_->Cancel();
                frozen_status = true;
            }
        }
        void Cancel() {
            frozen_status = false;
            chassis_executor_->Cancel();
        }

        BehaviorState Update() {
            return chassis_executor_->Update();
        }
        ~FrozenBehavior() = default;

    private:
        ChassisExecutor* const chassis_executor_;
        Blackboard* const blackboard_;
        bool frozen_status;
        
};
}

#endif //MOVE_DECISION_FROZEN_BEHAVIOR_H