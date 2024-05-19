#ifndef BULLET_HPP
#define BULLET_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "../../rm_common/include/error_code.hpp"
#include "../../rm_common/include/geometry.hpp"
#include "rm_decision_interfaces/msg/bullet_move.hpp"


namespace rmMultistage {
using rm_common::Point2D;

    class Bullet{
    public:
        Bullet(std::string shooter,double x,double y,double yaw,double speed);
        ~Bullet() = default;
        void Move(int frequency);
        std::string GetShooter()
        {return shooter_;}
        rm_decision_interfaces::msg::BulletMove GetBulletMove();
        bool ReachBoundary(int xmax, int ymax);
        Point2D GetPositionNow()
        {return position_now_;}
        Point2D GetPositionLast()
        {return position_last_;}
        float GetDistance()
        {return coveredDistance_;}
    private:
        std::string shooter_;
        double speed_;
        double yaw_;//form blue to red is 0, anciclockwise positive
        Point2D origin_;
        Point2D position_now_;
        Point2D position_last_;
        float coveredDistance_;
    };


}



#endif // BULLET_HPP
