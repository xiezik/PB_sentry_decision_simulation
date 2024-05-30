#ifndef __ROBOT_TF_HPP
#define __ROBOT_TF_HPP

#include <rclcpp/rclcpp.hpp>
#include "../../rm_common/include/error_code.hpp"
#include "../../rm_common/include/math.hpp"
#include "bullet.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "rm_decision_interfaces/srv/gimbal_aim.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"

namespace rmMultistage {

using Point2D=rm_common::Point2D;
using Polygon2D=rm_common::Polygon2D;

#define RobotWidth 0.5
#define RobotLength 0.6
#define AromrLength 0.13//real lenth(0.13) is to difficult to hit!
#define GimbalMaxSpeed 0.6
class RobotTF: public rclcpp::Node
    {
    public:
        RobotTF(std::string robot_name);
        std::string GetName()
        {return robot_name_;}
        float GetGimbalYaw()
        {return gimbal_yaw_;}
        float GetGimbalAbsoluteYaw();
        void SetGimbalAbsoluteYaw(double set_yaw_absolute);
        int BulletHitDetect(Bullet bullet,std::vector<Point2D> *points = nullptr);//front left right back 0 1 2 3
        bool GimbalMove();
        void PublishArmorTF();
        Point2D GetChassisSidesMid(int i)
        {return chassisSidesMid_[i];}
        tf2::Transform robot_tf;
        tf2::Transform armor_tf[4];//front left right back
        geometry_msgs::msg::PointStamped armor_point[4];
        bool isAlive;
        rm_decision_interfaces::msg::RobotStatus roboStatus_;

    private:
        std::string robot_name_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster>broadcaster_;

        rclcpp::Service<rm_decision_interfaces::srv::GimbalAim>::SharedPtr gimbal_aim_service_;
        rclcpp::Subscription<rm_decision_interfaces::msg::RobotStatus>::SharedPtr judgeStatus_sub_;

        float gimbal_yaw_;
        double gimbal_yaw_set_;
        float gimbal_yaw_absolute_;
        rm_common::ErrorInfo Init(std::string robot_name);
        
        void SetGimbalEncoderYaw(double yaw_set);
        
        void RobotStatusCallback(const rm_decision_interfaces::msg::RobotStatus::ConstSharedPtr& msg);
        
        bool GimbalAim_handle_function(
        rm_decision_interfaces::srv::GimbalAim_Request::SharedPtr &req,
        rm_decision_interfaces::srv::GimbalAim_Response::SharedPtr &res);
        Point2D chassisSidesMid_[4];
    };

}

#endif
