#ifndef TACTIC_LAYER_H
#define TACTIC_LAYER_H

#include <nav_msgs/msg/occupancy_grid.hpp>
// #include "io/io.h"
// #include "map_common.h"
#include "costmap_layer.hpp"
#include <nav_msgs/srv/detail/get_map__struct.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/subscription.hpp>
#include <rm_decision_interfaces/msg/detail/battle_position__struct.hpp>
#include "rm_decision_interfaces/msg/robot_position.hpp"
#include "rm_decision_interfaces/msg/battle_position.hpp"

namespace hero_costmap {

class TacticLayer : public CostmapLayer {

public:
 TacticLayer() {}
 virtual ~TacticLayer() {}
 virtual void OnInitialize();
 virtual void Activate();
 virtual void Deactivate();
 virtual void Reset();
 virtual void UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
 virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                          double* max_x, double* max_y);
 virtual void MatchSize() ;

private:
 std::string enemy_name_[2];
 rm_decision_interfaces::msg::BattlePosition battle_position_;
 rclcpp::Subscription<rm_decision_interfaces::msg::BattlePosition>::SharedPtr battle_position_sub_;
 rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr static_map_srv_;
 nav_msgs::msg::OccupancyGrid map_;

 bool battleinfo_received_;
 signed char *dirmap_;
 void BattlePositionCallback(const rm_decision_interfaces::msg::BattlePosition::SharedPtr &msg);
 rm_decision_interfaces::msg::RobotPosition FindRobotPosition(std::string robot_name);
 bool GetStaticMap();
 void UpdateTacticCost(std::string robot_name);
 void ResetDirmap();
 double GetTacticCost(double distance, int ammo);
};
}
#endif // TACTIC_LAYER_H
