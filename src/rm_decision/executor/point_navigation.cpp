#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "chassis_executor.hpp"
// #include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rm_decision_interfaces/action/detail/global_planner__struct.hpp>
#include <rm_decision_interfaces/action/detail/local_planner__struct.hpp>
#include "rm_decision_interfaces/action/global_planner.hpp"
#include "rm_decision_interfaces/action/local_planner.hpp"
namespace hero_decision{


class PointNavigation: public rclcpp::Node{
  public:
  using GlobalPlanner = rm_decision_interfaces::action::GlobalPlanner;
  using LocalPlanner = rm_decision_interfaces::action::LocalPlanner;
  using GoalHandleGlobalPlanner = rclcpp_action::ClientGoalHandle<GlobalPlanner>;
  using GoalHandleLocalPlanner = rclcpp_action::ClientGoalHandle<LocalPlanner>;
  PointNavigation(const rclcpp::NodeOptions & options);
  ~PointNavigation()=default;

  rm_decision_interfaces::action::GlobalPlanner::Goal global_planner_goal_;
  rm_decision_interfaces::action::LocalPlanner::Goal local_planner_goal_;

  rclcpp_action::Client<rm_decision_interfaces::action::GlobalPlanner>::SharedPtr global_planner_client_;
  rclcpp_action::Client<rm_decision_interfaces::action::LocalPlanner>::SharedPtr local_planner_client_;
  // ros::Subscriber sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  void GlobalPlannerFeedbackCallback(const GoalHandleGlobalPlanner::SharedPtr, const std::shared_ptr<const GlobalPlanner::Feedback>& global_planner_feedback);
  void chatterCallback(const geometry_msgs::msg::PoseStamped &goal);
  double set_yaw;
  double set_yaw_speed_;

};

PointNavigation::PointNavigation(const rclcpp::NodeOptions & options) : Node("point_nav_client", options)
{

  this->global_planner_client_ = rclcpp_action::create_client<rm_decision_interfaces::action::GlobalPlanner>(
      this,
      "global_planner_goal");
  this->local_planner_client_ = rclcpp_action::create_client<rm_decision_interfaces::action::LocalPlanner>(
      this,
      "local_planner_goal");


  if(this->get_namespace().c_str()!="/"&&this->get_namespace().size()>5)
  {
      sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(this->get_namespace().substr(1) + "/move_base_simple/goal", 
    rclcpp::SensorDataQoS(), 
    std::bind(&PointNavigation::chatterCallback, this, std::placeholders::_1));
  }
  else
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 
    rclcpp::SensorDataQoS(), 
    std::bind(&PointNavigation::chatterCallback, this, std::placeholders::_1));


  // RCLCPP_INFO(this->get_logger("PointNavigation"),"PointNavigation start!");
  // global_planner_client_.waitForServer();
  // ROS_INFO("Global planer server start!");
  // local_planner_client_.waitForServer();
  // ROS_INFO("Local planer server start!");
}

void PointNavigation::chatterCallback(const geometry_msgs::msg::PoseStamped &goal)
{
  global_planner_goal_.goal = goal;
  auto send_goal_options = rclcpp_action::Client<rm_decision_interfaces::action::GlobalPlanner>::SendGoalOptions();
  send_goal_options.feedback_callback =
      std::bind(&PointNavigation::GlobalPlannerFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  global_planner_client_->async_send_goal(global_planner_goal_,send_goal_options);

	RCLCPP_INFO(rclcpp::get_logger("PointNavigation"),"goal set! ");
}



void PointNavigation::GlobalPlannerFeedbackCallback(const GoalHandleGlobalPlanner::SharedPtr, const std::shared_ptr<const GlobalPlanner::Feedback>& global_planner_feedback){
  if (!global_planner_feedback->path.poses.empty()) {
    //ROS_INFO("global_planner_feedback");
    local_planner_goal_.route = global_planner_feedback->path;
    local_planner_client_->async_send_goal(local_planner_goal_);
  }

}



}
RCLCPP_COMPONENTS_REGISTER_NODE(hero_decision::PointNavigation)
// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "point_navigation");

// 	auto PointNavigation = new hero_decision::PointNavigation;



// 	/**
// 	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
// 	 * callbacks will be called from within this thread (the main one).  ros::spin()
// 	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
// 	 */
//   ros::spin();

// 	return 0;
// }
