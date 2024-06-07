#ifndef DEBUFF_LAYER_H
#define DEBUFF_LAYER_H

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/subscription.hpp>
#include <rm_decision_interfaces/msg/detail/buffinfo__struct.hpp>
#include <tf2_ros/buffer.h>
// #include "io/io.h"
// #include "map_common.h"
#include "costmap_layer.hpp"
#include "rm_decision_interfaces/msg/buffinfo.hpp"
#include "tf2_ros/transform_listener.h"
// #include "tf/tf.h"
namespace hero_costmap {

class DebuffLayer : public CostmapLayer {

 public:
  DebuffLayer() {}
  virtual ~DebuffLayer() {}
  virtual void OnInitialize();
  virtual void Activate();
  virtual void Deactivate();
  virtual void Reset();
  virtual void UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                           double* max_x, double* max_y);

 private:
 rclcpp::Subscription<rm_decision_interfaces::msg::Buffinfo>::SharedPtr buff_info_sub_;
  rm_decision_interfaces::msg::Buffinfo buffInfo_;
  bool buffinfo_received_;
  double RFID_F_x[6];
  double RFID_F_y[6];
  double RFID_height;
  double RFID_width;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  double inflation_;
  void GetParam(rclcpp::Node *nh);
  void BuffInfoCallback(const rm_decision_interfaces::msg::Buffinfo::SharedPtr &msg);
};

}












#endif // TACTIC_LAYER_H
