#ifndef HERO_DECISION_CHASSIS_EXECUTOR_H
#define HERO_DECISION_CHASSIS_EXECUTOR_H
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/service.hpp>

#include "rm_decision_interfaces/action/global_planner.hpp"
#include "rm_decision_interfaces/action/local_planner.hpp"
#include "rm_decision_interfaces/msg/twist_accel.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "../behavior_tree/behavior_state.h"

namespace hero_decision{
/***
 * @brief Chassis Executor to execute different abstracted task for chassis module
 */
class ChassisExecutor : public rclcpp::Node{

 
 public:
   using GlobalPlanner = rm_decision_interfaces::action::GlobalPlanner;
  using LocalPlanner = rm_decision_interfaces::action::LocalPlanner;
  using GoalHandleGlobalPlanner = rclcpp_action::ClientGoalHandle<GlobalPlanner>;
  using GoalHandleLocalPlanner = rclcpp_action::ClientGoalHandle<LocalPlanner>;
  /**
   * @brief Chassis execution mode for different tasks
   */
  enum class ExcutionMode{
    IDLE_MODE,            ///< Default idle mode with no task
    GOAL_MODE,            ///< Goal-targeted task mode using global and local planner
    SPEED_MODE,           ///< Velocity task mode
    SPEED_WITH_ACCEL_MODE ///< Velocity with acceleration task mode
  };
  /**
   * @brief Constructor of ChassisExecutor
   */
  ChassisExecutor(const rclcpp::NodeOptions & options);
  /**
   * @brief Execute the goal-targeted task using global and local planner with actionlib
   * @param goal Given taget goal
   */
  void Execute(const geometry_msgs::msg::PoseStamped &goal);
  /**
   * @brief Execute the velocity task with publisher
   * @param twist Given velocity
   */
  void Execute(const geometry_msgs::msg::Twist &twist);
  /**
   * @brief Execute the velocity with acceleration task with publisher
   * @param twist_accel Given velocity with acceleration
   */
  void Execute(const rm_decision_interfaces::msg::TwistAccel &twist_accel);
  /**
   * @brief Update the current chassis executor state
   * @return Current chassis executor state(same with behavior state)
   */
  BehaviorState Update();
  /**
   * @brief Cancel the current task and deal with the mode transition
   */
  void Cancel();



 private:
  /***
   * @brief Global planner actionlib feedback callback function to send the global planner path to local planner
   * @param global_planner_feedback  Global planner actionlib feedback, which mainly consists of global planner path output
   */
  void GlobalPlannerFeedbackCallback(const GoalHandleGlobalPlanner::SharedPtr, const std::shared_ptr<const GlobalPlanner::Feedback>& global_planner_feedback);
  void GlobalPlannerGoalResponseCallback(const GoalHandleGlobalPlanner::SharedPtr& goal_handle);
  void GlobalPlannerResultCallback(const GoalHandleGlobalPlanner::WrappedResult & result);
  //! execution mode of the executor
  ExcutionMode execution_mode_;
  //! execution state of the executor (same with behavior state)
  BehaviorState execution_state_;

  //! global planner actionlib client
  // actionlib::SimpleActionClient<rm_decision_interfaces::action::GlobalPlanner> global_planner_client_;
  //! local planner actionlib client
  // actionlib::SimpleActionClient<rm_decision_interfaces::action::LocalPlanner> local_planner_client_;

  //global planner action client
  rclcpp_action::Client<GlobalPlanner>::SharedPtr global_planner_action_client_;
  // typedef actionlib::SimpleActionClient<rm_decision_interfaces::action::LocalPlanner> LocalActionClient;
  rclcpp_action::Client<LocalPlanner>::SharedPtr local_planner_action_client_;
  //! global planner actionlib goal
  GlobalPlanner::Goal global_planner_goal_;
  //! local planner actionlib goal
  LocalPlanner::Goal local_planner_goal_;


  //! velocity control publisher in ROS2
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  //! zero twist in form of ROS geometry_msgs::Twist
  geometry_msgs::msg::Twist zero_twist_;

  //! velocity with accel publisher in ROS2
  rclcpp::Publisher<rm_decision_interfaces::msg::TwistAccel>::SharedPtr cmd_vel_acc_pub_;
  
  //! zero twist with acceleration in form of ROS2 rm_decision_interfaces::msg::TwistAccel
  rm_decision_interfaces::msg::TwistAccel zero_twist_accel_;


};
}


#endif //HERO_DECISION_CHASSIS_EXECUTOR_H