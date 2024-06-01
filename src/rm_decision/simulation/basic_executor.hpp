#ifndef __BASIC_EXECUTOR_H
#define __BASIC_EXECUTOR_H

#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/robot_position.hpp"
#include "rm_decision_interfaces/msg/battle_position.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/srv/detail/get_map__struct.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include "rm_decision_interfaces/srv/shoot_cmd.hpp"
#include "rm_decision_interfaces/srv/gimbal_aim.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_decision_interfaces/srv/basic_executor.hpp"
#include "rm_decision_interfaces/msg/basic_executor_status.hpp"

namespace hero_decision{

const std::string RobotName[4]={"robot_0","robot_1","robot_2","robot_3"};
const double MaxShootRange = 4.5;
const double I_OUT_YAW = 5;
const int LoopTimeMs = 50;

enum class BasicExecutorState {
  MOVE_TO_POSITION,
  ATTACK_ROBOT,
  IDLE,
};

class BasicExecutor : public rclcpp::Node
{
public:
  BasicExecutor(const rclcpp::NodeOptions & options);
  int CanShootRobot(std::string robot_name);
  bool AimRobot(std::string robot_name);
  bool EngageRobot(std::string robot_name);
  std::string FindClosetAimableEnemy();
  void YawControlLoop();
  bool ApproachEnemy(std::string robot_name,double contact_distance);
  void FaceRobot(std::string robot_name);
  void MoveToPosition(double x,double y);
  void FSM_handler();
  void TestBehaviour();
  std::string GetName()
  {return my_name_;}
private:
  std::string my_name_;
  rclcpp::Subscription<rm_decision_interfaces::msg::BattlePosition>::SharedPtr battle_position_sub_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr static_map_srv_;
  rclcpp::Client<rm_decision_interfaces::srv::ShootCmd>::SharedPtr shoot_client_;
  rclcpp::Client<rm_decision_interfaces::srv::GimbalAim>::SharedPtr gimbal_aim_client_;
  nav_msgs::msg::OccupancyGrid map_;
  rm_decision_interfaces::msg::BattlePosition battle_position_;

  hero_decision::BasicExecutorState state_;
  void Init();
  void BattlePositionCallback(const rm_decision_interfaces::msg::BattlePosition::ConstSharedPtr& msg);
  void YawSetCallback(const std_msgs::msg::Float64::ConstSharedPtr& msg);
  bool GetStaticMap();
  bool GimbalAimPoint(double x,double y);
  bool Shoot();
  bool isAlive(std::string robot_name);
  double set_yaw;
  double set_yaw_speed_;
  bool yaw_received_;
  bool yaw_control_received_;

  std::string target_enemy_;
  std::string attacking_target;
  double move_x;
  double move_y;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
  rclcpp::Publisher<rm_decision_interfaces::msg::BasicExecutorStatus>::SharedPtr basic_executor_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_set_sub_;
  rclcpp::Service<rm_decision_interfaces::srv::BasicExecutor>::SharedPtr basic_executor_server_;
  rm_decision_interfaces::msg::RobotPosition FindRobotPosition(std::string robot_name);
  bool BasicExecutor_handle_function(rm_decision_interfaces::srv::BasicExecutor::Request &req,
  rm_decision_interfaces::srv::BasicExecutor::Response &res);
  void PublishStatus();
  std::string saying;

};



}
#endif // __BASIC_EXECUTOR_H
