#include "tactic_costmap_plugin/tactic_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "../../rm_common/include/math.hpp"
#include "../../rm_common/include/rm_map.hpp"
#include <chrono>
#include <rclcpp/logger.hpp>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace tactic_costmap_plugin
{

TacticLayer::TacticLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
TacticLayer::onInitialize()
{
  node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  need_recalculation_ = false;
  current_ = true;
  battleinfo_received_ = false;
  dirmap_ = NULL;

  battle_position_sub_ = node->create_subscription<rm_decision_interfaces::msg::BattlePosition>(
      "/simu_decision_info/battle_position", 100,
      std::bind(&TacticLayer::BattlePositionCallback, this, std::placeholders::_1));
  static_map_srv_ = node->create_client<nav_msgs::srv::GetMap>("/static_map");

  RCLCPP_INFO(rclcpp::get_logger("tactic_cost"), "[TacticLayer]Initialized");

  while (!battleinfo_received_) {
    RCLCPP_INFO(rclcpp::get_logger("tactic_cost"), "[TacticLayer]Waiting battle info");
    // rclcpp::spin_some(node);
  }

  TacticLayer::matchSize();
  // default_value_ = FREE_SPACE;
  // is_enabled_ = true;
  // is_current_ = true;
  std::string current_namespace = node->get_namespace();
  if(current_namespace.substr(2)=="blue_decision" ||current_namespace.substr(2)=="robot_0"||current_namespace.substr(2) == "robot_1")
  {
    enemy_name_[0] = "robot_2";
    enemy_name_[1] = "robot_3";
  }
  else if(current_namespace.substr(2)=="red_decision" ||current_namespace.substr(2)=="robot_2"||current_namespace.substr(2) == "robot_3")
  {
    enemy_name_[0] = "robot_0";
    enemy_name_[1] = "robot_1";
  }
  else
  {
    RCLCPP_ERROR (rclcpp::get_logger("tactic_cost"),"tactic layer name error:%s",name_.c_str());
    // is_enabled_ = false;
  }

  GetStaticMap();
  //从静态地图中获取原点
  origin_x_ = map_.info.origin.position.x;
  origin_y_ = map_.info.origin.position.y;
  resolution_ = map_.info.resolution;
  // costmap_ = map_.data;


}

double  TacticLayer::GetTacticCost(double distance, int ammo)
{
  return 60.0/(distance) * (double)std::min(ammo + 3,20) / 20.0;
}

void TacticLayer::UpdateTacticCost(std::string robot_name)
{
  //ROS_INFO("Update enemy: %s cost",robot_name.c_str());
  if(FindRobotPosition(robot_name).health==0)
    return;
  double enemy_x,enemy_y;
  enemy_x = FindRobotPosition(robot_name).position.x;
  enemy_y = FindRobotPosition(robot_name).position.y;
  for(unsigned int i =0;i<size_x_*size_y_;i++)
  {
    unsigned int cell_x,cell_y;
    double map_x,map_y;
    Index2Cells(i,cell_x,cell_y);
    Map2World(cell_x,cell_y,map_x,map_y);
    double distance = rm_common::PointDistance(enemy_x,enemy_y,map_x,map_y);
    distance = std::max(0.8,distance);
    if(distance < 5)
    {
      if(!rm_common::LineSegmentIsIntersectMapObstacle(&map_,enemy_x,enemy_y,map_x,map_y,50))
      {
        if(dirmap_[i] == -128)
        {
          map_.data[i] += GetTacticCost(distance,FindRobotPosition(robot_name).ammo);
          dirmap_[i] = (std::atan2(enemy_y - map_y,enemy_x - map_x) / M_PI) * 127;
        }
        else {
          signed char new_dir = (std::atan2(enemy_y - map_y,enemy_x - map_x) / M_PI) * 127;
          double dir_diff = (std::min(std::abs((int)new_dir - (int)dirmap_[i]),254 - std::abs((int)new_dir - (int)dirmap_[i])))*M_PI/127.0;
          map_.data[i] = std::min((unsigned char)(((double)map_.data[i] + GetTacticCost(distance,FindRobotPosition(robot_name).ammo)) * (1.5 - 0.5 * std::cos(dir_diff) )),INSCRIBED_INFLATED_OBSTACLE);
        }
      }
      else
      {
        map_.data[i] += GetTacticCost(distance,FindRobotPosition(robot_name).ammo) * 0.3;
      }
    }
  }
}

rm_decision_interfaces::msg::RobotPosition TacticLayer::FindRobotPosition(std::string robot_name)
{
  for (auto it = battle_position_.robots_position.begin(); it != battle_position_.robots_position.end(); ++it) {
    if((*it).robot_name == robot_name)
    {
      return *(it);
    }

  }
 // ROS_ERROR("no %s",robot_name.c_str());
 return rm_decision_interfaces::msg::RobotPosition();
  // return hero_msgs::RobotPosition();
}

void TacticLayer::Index2Cells(unsigned int index, unsigned int &mx, unsigned int &my){
    my = index / size_x_;
    mx = index - (my * size_x_);
  }

void TacticLayer::Map2World(unsigned int mx, unsigned int my, double &wx, double &wy){
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool TacticLayer::GetStaticMap()
{
  if (!static_map_srv_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("tactic_cost"), "[TacticLayer]Failed to connect to /static_map service");
    return false;
  }

  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto result = static_map_srv_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("tactic_cost"), "[TacticLayer]Received Static Map");
    map_ = result.get()->map;
    return true;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("tactic_cost"), "[TacticLayer]Failed to get static map");
    return false;
  }
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
TacticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
TacticLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "TacticLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the tactic costmap is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
TacticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Simply computing one-by-one cost per each cell
  int gradient_index;
  for (int j = min_j; j < max_j; j++) {
    // Reset gradient_index each time when reaching the end of re-calculated window
    // by OY axis.
    gradient_index = 0;
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      // setting the gradient cost
      unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
      if (gradient_index <= GRADIENT_SIZE) {
        gradient_index++;
      } else {
        gradient_index = 0;
      }
      master_array[index] = cost;
    }
  }
}

void TacticLayer::BattlePositionCallback(const rm_decision_interfaces::msg::BattlePosition::ConstSharedPtr& msg)
{
  battle_position_ = *msg;
  battleinfo_received_ = true;
}

}  // namespace tactic_costmap_plugin

// This is the macro allowing a tactic_costmap_plugin::TacticLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tactic_costmap_plugin::TacticLayer, nav2_costmap_2d::Layer)
