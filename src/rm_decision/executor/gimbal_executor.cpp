#include "gimbal_executor.hpp"
namespace hero_decision{
GimbalExecutor::GimbalExecutor(const rclcpp::NodeOptions & options): Node("gimbal_executor", options),
excution_mode_(ExcutionMode::IDLE_MODE),
execution_state_(BehaviorState::IDLE){
  cmd_gimbal_angle_pub_ = 
    this->create_publisher<rm_decision_interfaces::msg::GimbalAngle>("cmd_gimbal_angle", 1);
  cmd_gimbal_rate_pub_ =
    this->create_publisher<rm_decision_interfaces::msg::GimbalRate>("cmd_gimbal_rate", 1);

}

void GimbalExecutor::Execute(const rm_decision_interfaces::msg::GimbalAngle &gimbal_angle){
  excution_mode_ = ExcutionMode::ANGLE_MODE;
  cmd_gimbal_angle_pub_->publish(gimbal_angle);
}

void GimbalExecutor::Execute(const rm_decision_interfaces::msg::GimbalRate &gimbal_rate){
  excution_mode_ = ExcutionMode::RATE_MODE;
  cmd_gimbal_rate_pub_->publish(gimbal_rate);
}

BehaviorState GimbalExecutor::Update(){
  switch (excution_mode_){
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::ANGLE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::RATE_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      RCLCPP_ERROR(rclcpp::get_logger("GimbalExecutor"),"Wrong Execution Mode");
  }
  return execution_state_;
}

void GimbalExecutor::Cancel(){
  switch (excution_mode_){
    case ExcutionMode::IDLE_MODE:
      RCLCPP_WARN(rclcpp::get_logger("GimbalExecutor"),"Nothing to be canceled.");
      break;

    case ExcutionMode::ANGLE_MODE:
      cmd_gimbal_rate_pub_->publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::RATE_MODE:
      cmd_gimbal_rate_pub_->publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    default:
      RCLCPP_ERROR(rclcpp::get_logger("GimbalExecutor"),"Wrong Execution Mode");
  }

}
}