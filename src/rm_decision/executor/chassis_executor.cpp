#include "chassis_executor.hpp"
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <rm_decision_interfaces/action/detail/global_planner__struct.hpp>

namespace hero_decision{

ChassisExecutor::ChassisExecutor(const rclcpp::NodeOptions & options) : Node("chassis_executor", options),
execution_mode_(ExcutionMode::IDLE_MODE), execution_state_(BehaviorState::IDLE)
{
  cmd_vel_acc_pub_ = this->create_publisher<rm_decision_interfaces::msg::TwistAccel>(
    "cmd_vel_acc", rclcpp::SensorDataQoS());
  cmd_vel_pub_     = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS());
  this->global_planner_action_client_ = rclcpp_action::create_client<rm_decision_interfaces::action::GlobalPlanner>(
      this,
      "global_planner_goal");
  this->local_planner_action_client_ = rclcpp_action::create_client<rm_decision_interfaces::action::LocalPlanner>(
      this,
      "local_planner_goal");
  // global_planner_client_.waitForServer();
  // RCLCPP_INFO(this->get_logger(),"Global planer server start!");
  // local_planner_client_.waitForServer();
  // RCLCPP_INFO(this->get_logger(),"Local planer server start!");
}


void ChassisExecutor::GlobalPlannerGoalResponseCallback(const GoalHandleGlobalPlanner::SharedPtr& goal_handle){
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
  
}

void ChassisExecutor::GlobalPlannerFeedbackCallback(const GoalHandleGlobalPlanner::SharedPtr, const std::shared_ptr<const GlobalPlanner::Feedback>& global_planner_feedback){
  if (!global_planner_feedback->path.poses.empty()) {
    local_planner_goal_.route = global_planner_feedback->path;
    local_planner_action_client_->async_send_goal(local_planner_goal_);
  }
}

void ChassisExecutor::GlobalPlannerResultCallback(const GoalHandleGlobalPlanner::WrappedResult & result){
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

void ChassisExecutor::Execute(const geometry_msgs::msg::PoseStamped &goal){
  execution_mode_ = ExcutionMode::GOAL_MODE;
  global_planner_goal_.goal = goal;
  // global_planner_client_.sendGoal(global_planner_goal_,
  //                                 GlobalActionClient::SimpleDoneCallback(),
  //                                 GlobalActionClient::SimpleActiveCallback(),
  //                                 boost::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, std::placeholders::_1));
  
  auto send_goal_options = rclcpp_action::Client<rm_decision_interfaces::action::GlobalPlanner>::SendGoalOptions();
  send_goal_options.goal_response_callback = 
      std::bind(&ChassisExecutor::GlobalPlannerGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
      std::bind(&ChassisExecutor::GlobalPlannerResultCallback, this, std::placeholders::_1);
  this->global_planner_action_client_->async_send_goal(global_planner_goal_,send_goal_options);
}

void ChassisExecutor::Execute(const geometry_msgs::msg::Twist &twist){
  if( execution_mode_ == ExcutionMode::GOAL_MODE){
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_MODE;
  cmd_vel_pub_->publish(twist);
}

void ChassisExecutor::Execute(const rm_decision_interfaces::msg::TwistAccel &twist_accel){
  if( execution_mode_ == ExcutionMode::GOAL_MODE){
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_WITH_ACCEL_MODE;

  cmd_vel_acc_pub_->publish(twist_accel);
}

BehaviorState ChassisExecutor::Update(){
  rclcpp_action::ClientGoalHandle<rm_decision_interfaces::action::GlobalPlanner>::SharedPtr goal_handle;
  auto status = goal_handle->get_status();
  switch (execution_mode_){
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::GOAL_MODE:
      if (status == rclcpp_action::GoalStatus::STATUS_UNKNOWN) {
        RCLCPP_INFO(this->get_logger(), "Goal status: Unknown");
      } else if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
        RCLCPP_INFO(this->get_logger(), "Goal status: Accepted");
        execution_state_ = BehaviorState::RUNNING;
      } else if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
        RCLCPP_INFO(this->get_logger(), "Goal status: Executing");
        execution_state_ = BehaviorState::RUNNING;
      } else if (status == rclcpp_action::GoalStatus::STATUS_CANCELING) {
        RCLCPP_INFO(this->get_logger(), "Goal status: Canceling");
        execution_state_ = BehaviorState::RUNNING;
      } else if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal status: Succeeded");
        execution_state_ = BehaviorState::SUCCESS;
      } else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED) {
        RCLCPP_INFO(this->get_logger(), "Goal status: Aborted");
        execution_state_ = BehaviorState::FAILURE;
      } else if (status == rclcpp_action::GoalStatus::STATUS_CANCELED) {
        RCLCPP_INFO(this->get_logger(), "Goal status: Canceled");
        execution_state_ = BehaviorState::SUCCESS;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown goal status");
        execution_state_ = BehaviorState::FAILURE;
      }
      break;

    case ExcutionMode::SPEED_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      execution_state_ = BehaviorState::FAILURE;
      break;
      RCLCPP_ERROR(this->get_logger(), "Wrong Execution Mode");
  }
  return execution_state_;

};

void ChassisExecutor::Cancel(){
  switch (execution_mode_){
    case ExcutionMode::IDLE_MODE:
      RCLCPP_WARN(this->get_logger(),"Nothing to be canceled.");
      break;

    case ExcutionMode::GOAL_MODE:
      global_planner_action_client_->async_cancel_all_goals();
      local_planner_action_client_->async_cancel_all_goals();
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_MODE:
      cmd_vel_pub_->publish(zero_twist_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:
      cmd_vel_acc_pub_->publish(zero_twist_accel_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      usleep(50000);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(),"Wrong Execution Mode");
  }

}



}