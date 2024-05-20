#ifndef __ROBOT_PHYSICS_HPP
#define __ROBOT_PHYSICS_HPP


#include <functional>
#include <memory>
#include <nav_msgs/srv/detail/get_map__struct.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/service.hpp>
#include <rm_decision_interfaces/msg/detail/bullets_info__struct.hpp>
#include <rm_decision_interfaces/msg/detail/judge_sys_shoot_hit__struct.hpp>
#include <rm_decision_interfaces/srv/detail/shoot_cmd__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include "../../rm_common/include/error_code.hpp"
#include "../../rm_common/include/command_code.hpp"
#include "robot_tf.hpp"
#include "bullet.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/srv/get_map.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "rm_decision_interfaces/srv/judge_sys_control.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "rm_decision_interfaces/srv/shoot_cmd.hpp"
#include "rm_decision_interfaces/msg/robot_heat.hpp"
#include "rm_decision_interfaces/msg/robot_position.hpp"
#include "rm_decision_interfaces/msg/battle_position.hpp"
#include "rm_decision_interfaces/msg/bullets_info.hpp"
#include "rm_decision_interfaces/msg/judge_sys_shoot_hit.hpp"

#define DistributeYaw 0.03f
#define DistributeSpeed 2.0f
#define HeatControlLimit 200
namespace rmMultistage {
class  RobotPhysics: public rclcpp::Node{

public:
     RobotPhysics(const rclcpp::NodeOptions & options);
     bool AddRobot(RobotTF*  robot);

     void PublishGimbalYaw();

     RobotTF* FindRobot(std::string robot_name);
     void BulletJudge();
     void PublishBulletsInfo();
     void RobotShoot(std::string robot_name);
     void LetBulletsFly(int fre);
     int ArmorHitDetect();
     void SendHitRobotInfo(std::string robot_name, int armor_num);
     void SendShootRobotInfo(std::string robot_name);

     void RobotStatusCallback0(const rm_decision_interfaces::msg::RobotStatus::ConstSharedPtr& msg);
     void RobotStatusCallback1(const rm_decision_interfaces::msg::RobotStatus::ConstSharedPtr& msg);
     void RobotStatusCallback2(const rm_decision_interfaces::msg::RobotStatus::ConstSharedPtr& msg);
     void RobotStatusCallback3(const rm_decision_interfaces::msg::RobotStatus::ConstSharedPtr& msg);


     void GetParam();
     void RFID_detect();
     void SendJudgeSysRequest(std::string robot_name, int command);
     void SetRobotHeat( const rm_decision_interfaces::msg::RobotHeat::SharedPtr& msg,int index);
     void GimbalsMove();

     void PublishSimuDecisionInfo();
     void ShootSequence();
     bool need_shooting[4];
private:
    std::vector<RobotTF* > robots_;
    std::vector<Bullet* > bullets_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_[4];
    rclcpp::Subscription<rm_decision_interfaces::msg::RobotHeat>::SharedPtr roboHeat_ptr[4];
    std::shared_ptr<rm_decision_interfaces::msg::RobotHeat> robotHeat[4];
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gimbalYaw_pub_[4];
    rclcpp::Publisher<rm_decision_interfaces::msg::BulletsInfo>::SharedPtr bulletsInfo_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::BattlePosition>::SharedPtr simu_decision_info_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::JudgeSysShootHit>::SharedPtr shoot_hit_pub_;

    nav_msgs::msg::OccupancyGrid map_;
    rclcpp::Client<rm_decision_interfaces::srv::JudgeSysControl>::SharedPtr judgeSysClient_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr static_map_srv_;
    rclcpp::Service<rm_decision_interfaces::srv::ShootCmd>::SharedPtr shoot_service_;

     double RFID_F_x[6];
     double RFID_F_y[6];

     double RFID_height;
     double RFID_width;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;



    bool robot_tf_received_[4];
    rm_common::ErrorInfo Init();
    void PoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
    bool Shoot_handle_function(rm_decision_interfaces::srv::ShootCmd_Request::SharedPtr &req,
    rm_decision_interfaces::srv::ShootCmd_Response::SharedPtr &res);

};

}

#endif
