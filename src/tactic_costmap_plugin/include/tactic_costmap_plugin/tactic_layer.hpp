#ifndef TACTIC_LAYER_HPP_
#define TACTIC_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <memory>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/srv/detail/get_map__struct.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/subscription.hpp>
#include "rm_decision_interfaces/msg/battle_position.hpp"
#include "rm_decision_interfaces/msg/robot_position.hpp"

namespace tactic_costmap_plugin
{

class TacticLayer : public nav2_costmap_2d::Layer
{
public:
  TacticLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

private:
  std::string enemy_name_[2];
  rm_decision_interfaces::msg::BattlePosition battle_position_;
  rclcpp::Subscription<rm_decision_interfaces::msg::BattlePosition>::SharedPtr battle_position_sub_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr static_map_srv_;
  nav_msgs::msg::OccupancyGrid map_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;

  bool battleinfo_received_;
  signed char *dirmap_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_, origin_x_, origin_y_, resolution_;
  unsigned int size_x_;
  unsigned int size_y_;
  void BattlePositionCallback(const rm_decision_interfaces::msg::BattlePosition::ConstSharedPtr &msg);

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
  rm_decision_interfaces::msg::RobotPosition FindRobotPosition(std::string robot_name);
  void Map2World(unsigned int mx, unsigned int my, double &wx, double &wy);
  bool GetStaticMap();
  void UpdateTacticCost(std::string robot_name);
  void Index2Cells(unsigned int index, unsigned int &mx, unsigned int &my);
  double GetTacticCost(double distance, int ammo);
};

}  // namespace nav2_gradient_costmap_plugin

#endif  // GRADIENT_LAYER_HPP_
