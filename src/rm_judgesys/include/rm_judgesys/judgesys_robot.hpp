#ifndef __JUDGESYS_CONTROL_NODE_HPP_
#define __JUDGESYS_CONTROL_NODE_HPP_

#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/publisher.hpp>
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "rm_decision_interfaces/msg/robot_heat.hpp"
#include "../../rm_common/include/error_code.hpp"
#include "../../rm_common/include/command_code.hpp"

namespace rm_judgesys {

class JudgesysRobot : public rclcpp::Node{

  public:
  JudgesysRobot(std::string robot_num, std::string color);

  std::string GetColor()
        {return color_;}

         std::string GetName()
        {return robot_num_;}

        int GetHealth()
        {return health_;}

        int GetHeat()
        {return heat_;}
        
        bool IsAlive()
        {return is_alive_;}
        
        bool IsBegin()
        {return begin_;}
        
        int GetAmmo()
        {return ammo_;}

        void RawVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);
        void RawVelCallback_act(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);

        void PublishVel();
        void PublishInfo();
        
        void Shoot(int num, float speed);


        void BeHit(rm_common::JudgeSysCommand command);

        void DebuffMove();

        void DebuffShoot();

        void BuffReload();

        void BuffHeal();

        void Kill(){health_=0;}

        void Revive();

        void Reload()
        {
            ammo_+=50;
            if(ammo_>400)
                ammo_=400;
        }

        void Disarm()
        {ammo_ = 0;}

        void BuffAndDebuff(int fre);

        void Cool(int fre);

        void Reset();


        private:

        
        rm_common::ErrorInfo Init(std::string robot_num, std::string color);

        std::string color_;
        std::string robot_num_;

        int health_;
        float heat_;
        int cooling_rate_;
        int heat_cooling_limit_;
        int ammo_;
        bool is_alive_;

        bool is_forbidden_to_move_;
        bool is_forbidden_to_shoot_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr judgeVel_pub_ptr;
        rclcpp::Publisher<rm_decision_interfaces::msg::RobotStatus>::SharedPtr judgeStatus_pub_ptr;
        rclcpp::Publisher<rm_decision_interfaces::msg::RobotHeat>::SharedPtr judgeHeat_pub_ptr;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr judge_sub_ptr[2];
        int raw_cmd_selector_count;

        rm_decision_interfaces::msg::RobotStatus roboStatus_;
        rm_decision_interfaces::msg::RobotHeat roboHeat_;

        geometry_msgs::msg::Twist raw_cmd_vel_;
        geometry_msgs::msg::Twist output_cmd_vel_;

        bool new_cmd_acc_;
        bool begin_;

        float move_debuff_time_;
        float shoot_debuff_time_;
        rclcpp::Logger logger_{rclcpp::get_logger("rm_behavor_tree")};

};

} // namespace rm_judgesys


#endif  // __JUDGESYS_CONTROL_NODE_HPP_