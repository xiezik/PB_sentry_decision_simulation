#ifndef COLLECTIVE_DECISION_H
#define COLLECTIVE_DECISION_H

#include <rclcpp/rclcpp.hpp>
#include "rm_decision_interfaces/msg/robot_position.hpp"
#include "rm_decision_interfaces/msg/battle_position.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/srv/get_map.hpp>
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_decision_interfaces/srv/basic_executor.hpp"
#include "rm_decision_interfaces/msg/basic_executor_status.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "rm_decision_interfaces/msg/buffinfo.hpp"
#include "rm_decision_interfaces/msg/game_status.hpp"
#include "hero_costmap/costmap_interface.h"

namespace hero_decision{

const std::string RobotName[4]={"robot_0","robot_1","robot_2","robot_3"};

class Collective_decision
{
public:
  Collective_decision(std::string color);
  void Spin();
  typedef std::shared_ptr<hero_costmap::CostmapInterface> CostmapPtr;
  typedef std::shared_ptr<tf::TransformListener> TfPtr;

  enum TaskType{
    ATTACK_ROBOT_0 = 2000u,
    ATTACK_ROBOT_1 = 2001u,
    GET_AMMO = 2002u,
    GET_HEALTH = 2003u,
    GET_ENEMY_HEALTH = 2004u,
    FLEE = 2005u,
    IDLE = 2006u,
  };

  enum BuffType{
    FRIENDLY_HEAL = 0u,
    FRIENDLY_AMMO = 1u,
    ENEMY_HEAL = 2u,
    ENEMY_AMMO = 3u,
  };

  struct RobotTask
  {
    RobotTask(TaskType task_type) {type = task_type;benefit = 0; cost = 0;}
    RobotTask(){type = IDLE; benefit = 0; cost = 0;}
    std::vector<std::string> assigned_robots;
    TaskType type;
    double benefit;
    double cost;
  };

private:




  struct RobotStatus
  {
    RobotStatus() {task = NULL;}
    std::string name;
    int health;
    int ammo;
    double x,y;
    double combat_effectiveness;
    std::string saying;
   RobotTask *task;
   std::vector<RobotTask> candidate_tasks;
  };
  rclcpp::Node::SharedPtr nh_;
  std::string color_;
  std::string friendly_name_[2];
  std::string enemy_name_[2];
  rm_decision_interfaces::msg::Buffinfo buffInfo_;
  rm_decision_interfaces::msg::GameStatus gameStatus_;
  rclcpp::Subscription<rm_decision_interfaces::msg::BattlePosition>::SharedPtr battle_position_sub_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr static_map_srv_;

  rclcpp::Subscription<rm_decision_interfaces::msg::GameStatus>::SharedPtr gameStatus_sub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::Buffinfo>::SharedPtr buffInfo_sub_;

  rclcpp::Client<rm_decision_interfaces::srv::BasicExecutor>::SharedPtr basic_executor_cient_[2];

  nav_msgs::msg::OccupancyGrid map_;
  rm_decision_interfaces::msg::BattlePosition battle_position_;

  CostmapPtr costmap_ptr_;
  //！ Transform pointer
  TfPtr tf_ptr_;

  RobotStatus robot_status[2];
  RobotStatus enemy_status[2];
  enum SideType{
    FRIENDLY_ONLY = 1000u,
    ENEMY_ONLY = 1001u,
    BOTH_SIDE= 1003u,
  };
  double RFID_F_x[6];
  double RFID_F_y[6];

  double RFID_height;
  double RFID_width;
  bool GetStaticMap();
  void GameStatusCallback(const rm_decision_interfaces::msg::GameStatus::SharedPtr msg);
  void BuffInfoCallback(const rm_decision_interfaces::msg::Buffinfo::SharedPtr msg);
  void BattlePositionCallback(const rm_decision_interfaces::msg::BattlePosition::SharedPtr msg);
  void MoveToPosition(int robot_num, double x,double y);
  void AttackRobot(int robot_num,int enemy_num);
  void GoGetBuff(int robot_num,int buff_num);
  int GetBuffRFIDNum(BuffType buff_type);
  void Init();
  void GetParam(rclcpp::Node::SharedPtr nh);
  rm_decision_interfaces::msg::RobotPosition FindRobotPosition(std::string robot_name);
  int FindClosestRobotToPosition(SideType specific_side,double x,double y);
  void FleeBehaviour(int robot_num);
  void RobotStatusUpdate();
  void TasksManager();
  void RobotsExecutor();
  void TaskExecutor(int robot_num, RobotTask *task);
  void CalculateCostBenefitCost();
  void TaskAssign();
  bool GetOptimalEngagePosition(int robot_num, int enemy_num, double &optimal_x, double &optimal_y);
  double GetContactDistanceCost(double distance,double optimal_distance);
  //bool TaskSortFun(const RobotTask &t1,const RobotTask &t2);
};

}
#endif // COLLECTIVE_DECISION_H
