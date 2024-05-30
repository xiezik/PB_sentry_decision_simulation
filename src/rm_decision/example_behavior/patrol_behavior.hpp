#ifndef HERO_DECISION_PATROL_BEHAVIOR_H
#define HERO_DECISION_PATROL_BEHAVIOR_H

// #include "io/io.h"

#include "../blackboard/blackboard.hpp"
#include "../executor/chassis_executor.hpp"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.hpp"

#include "line_iterator.hpp"
#include <rclcpp/logger.hpp>

namespace hero_decision {
class PatrolBehavior {
 public:
  PatrolBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {

    patrol_count_ = 0;
    point_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      RCLCPP_ERROR(rclcpp::get_logger("PatrolBehavior"),"%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();

    std::cout << "state: " << (int)(executor_state) << std::endl;

    if (executor_state != BehaviorState::RUNNING) {

      if (patrol_goals_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("PatrolBehavior"),"patrol goal is empty");
        return;
      }

      std::cout << "send goal" << std::endl;
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;

    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    hero_decision::DecisionConfig decision_config;
    if (!hero_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    point_size_ = (unsigned int)(decision_config.point().size());
    patrol_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++) {
      patrol_goals_[i].header.frame_id = "map";
      patrol_goals_[i].pose.position.x = decision_config.point(i).x();
      patrol_goals_[i].pose.position.y = decision_config.point(i).y();
      patrol_goals_[i].pose.position.z = decision_config.point(i).z();

      tf2::Quaternion quaternion = tf2::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                              decision_config.point(i).pitch(),
                                                              decision_config.point(i).yaw());
      patrol_goals_[i].pose.orientation.x = quaternion.x();
      patrol_goals_[i].pose.orientation.y = quaternion.y();
      patrol_goals_[i].pose.orientation.z = quaternion.z();
      patrol_goals_[i].pose.orientation.w = quaternion.w();
    }

    return true;
  }

  ~PatrolBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::msg::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;

};
}

#endif //HERO_DECISION_PATROL_BEHAVIOR_H
