//
// Created by autocar on 18-12-21.
//

#ifndef MOVE_DECISION_BEHAVIOR_STATE_H
#define MOVE_DECISION_BEHAVIOR_STATE_H
namespace roborts_decision {
/**
 * @brief Behavior state
 */
enum class BehaviorState {
  RUNNING,   ///< Running state in process
  SUCCESS,   ///< Success state as result
  FAILURE,   ///< Failure state as result
  IDLE,      ///< Idle state, state as default or after cancellation
};
}
#endif //ROBORTS_DECISION_BEHAVIOR_STATE_H
