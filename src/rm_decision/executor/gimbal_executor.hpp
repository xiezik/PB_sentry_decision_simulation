#ifndef HERO_DECISION_GIMBAL_EXECUTOR_H
#define HERO_DECISION_GIMBAL_EXECUTOR_H
#include "rclcpp/rclcpp.hpp"

#include "rm_decision_interfaces/msg/gimbal_angle.hpp"
#include "rm_decision_interfaces/msg/gimbal_rate.hpp"

#include "../behavior_tree/behavior_state.h"
namespace hero_decision{
/***
 * @brief Gimbal Executor to execute different abstracted task for gimbal module
 */
class GimbalExecutor : public rclcpp::Node{
 public:
  /**
   * @brief Gimbal execution mode for different tasks
   */
  enum class ExcutionMode{
    IDLE_MODE,   ///< Default idle mode with no task
    ANGLE_MODE,  ///< Angle task mode
    RATE_MODE    ///< Rate task mode
  };
  /**
   * @brief Constructor of GimbalExecutor
   */
  GimbalExecutor(const rclcpp::NodeOptions & options);
  /***
   * @brief Execute the gimbal angle task with publisher
   * @param gimbal_angle Given gimbal angle
   */
  void Execute(const rm_decision_interfaces::msg::GimbalAngle &gimbal_angle);
  /***
   * @brief Execute the gimbal rate task with publisher
   * @param gimbal_rate Given gimbal rate
   */
  void Execute(const rm_decision_interfaces::msg::GimbalRate &gimbal_rate);
  /**
   * @brief Update the current gimbal executor state
   * @return Current gimbal executor state(same with behavior state)
   */
  BehaviorState Update();
  /**
   * @brief Cancel the current task and deal with the mode transition
   */
  void Cancel();

 private:
  //! execution mode of the executor
  ExcutionMode excution_mode_;
  //! execution state of the executor (same with behavior state)
  BehaviorState execution_state_;

  //! gimbal rate control publisher in ROS
  rclcpp::Publisher<rm_decision_interfaces::msg::GimbalRate>::SharedPtr cmd_gimbal_rate_pub_;
  //! zero gimbal rate in form of ROS hero_msgs::GimbalRate
  rm_decision_interfaces::msg::GimbalRate zero_gimbal_rate_;

  //! gimbal angle control publisher in ROS
  rclcpp::Publisher<rm_decision_interfaces::msg::GimbalAngle>::SharedPtr cmd_gimbal_angle_pub_;


};
}


#endif //HERO_DECISION_GIMBAL_EXECUTOR_H