
#include "rm_judgesys/judgesys_robot.hpp"
#include "rm_judgesys/judgesys_control_node.hpp"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <string>


namespace rm_judgesys{

JudgesysRobot::JudgesysRobot(std::string robot_num, std::string color):
Node("JudgesysRobot"),
is_alive_(true),
is_forbidden_to_move_(false),
raw_cmd_selector_count(50),
new_cmd_acc_(false),
begin_(false){
   if (Init(robot_num,color).IsOK()) {
    RCLCPP_INFO(logger_,"%s robot:%s initialization completed.",color.c_str(),robot_num.c_str());
  } else {
    RCLCPP_ERROR(logger_,"%s robot:%s Initialization failed.",color.c_str(),robot_num.c_str());
  }
}

rm_common::ErrorInfo JudgesysRobot::Init(std::string robot_num, std::string color){
	if(color != "red" && color != "blue")
	{
		RCLCPP_ERROR(logger_,"robot color can only be red or blue.");
		return rm_common::ErrorInfo(rm_common::ErrorCode::GP_INITILIZATION_ERROR,
						"robot color can only be red or blue.");
	}
	color_ = color;
	robot_num_ = robot_num;
    Reset();
    judgeVel_pub_ptr = this->create_publisher<geometry_msgs::msg::Twist>(robot_num + "/" + "cmd_vel", 5);
    judgeStatus_pub_ptr = this->create_publisher<rm_decision_interfaces::msg::RobotStatus>("judgeSysInfo/" + robot_num + "/status",5);
    judgeHeat_pub_ptr = this->create_publisher<rm_decision_interfaces::msg::RobotHeat>("judgeSysInfo/" + robot_num + "/heat_power",5);

    judge_sub_ptr[0] = this->create_subscription<geometry_msgs::msg::Twist>(robot_num + "/" + "cmd_vel_raw", 
    rclcpp::SensorDataQoS(), 
    std::bind(&JudgesysRobot::RawVelCallback, this, std::placeholders::_1));

    judge_sub_ptr[1] = this->create_subscription<geometry_msgs::msg::Twist>(robot_num + "/" + "cmd_vel_raw_act", 
    rclcpp::SensorDataQoS(), 
    std::bind(&JudgesysRobot::RawVelCallback_act, this, std::placeholders::_1));
	
    return rm_common::ErrorInfo(rm_common::ErrorCode::OK);
}

void JudgesysRobot::RawVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
  raw_cmd_selector_count ++;
  if(raw_cmd_selector_count<50)
    return;
	if (!begin_) {
    begin_ = true;
        RCLCPP_INFO(logger_,"[rm_judgesys]%s robot:%s raw cmd vel received!",color_.c_str(),robot_num_.c_str());
  }
	new_cmd_acc_ = true;
	raw_cmd_vel_ = *msg;
}

void JudgesysRobot::RawVelCallback_act(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
  if (!begin_) {
    begin_ = true;
        RCLCPP_INFO(logger_,"[rm_judgesys]%s robot:%s raw cmd vel(act) received!",color_.c_str(),robot_num_.c_str());
  }
  new_cmd_acc_ = true;
  raw_cmd_vel_ = *msg;
  raw_cmd_selector_count = 0;
}

void JudgesysRobot::BuffReload()
{
    ammo_ += JudgesysControl::ReloadAmmo;
    RCLCPP_INFO(logger_,"[rm_judgesys]%s %s Buff Reload! ammo + 200.",color_.c_str(),robot_num_.c_str());
}

void JudgesysRobot::BuffHeal()
{
    if(!IsAlive())
        return;
    
    health_ += JudgesysControl::RecoveredHealth;
    if(health_ > JudgesysControl::MaxHealth )
        health_ = JudgesysControl::MaxHealth;
    
    RCLCPP_INFO(logger_,"[rm_judgesys]%s %s Buff Heal! Hp + 200.",color_.c_str(),robot_num_.c_str());
}

void JudgesysRobot::Revive()
{
    health_ = JudgesysControl::MaxHealth;
}
void JudgesysRobot::PublishVel()
{
	if(!begin_)
		return;
	if(health_<0)
	{
		health_ = 0;
	}
	is_alive_ = health_ > 0;
	if(is_forbidden_to_move_||!is_alive_)
	{
		output_cmd_vel_.linear.x = 0;
		output_cmd_vel_.linear.y = 0;
		output_cmd_vel_.angular.z = 0;
	}
	else
	{
		output_cmd_vel_.linear.x = raw_cmd_vel_.linear.x;
		output_cmd_vel_.linear.y = raw_cmd_vel_.linear.y;
		output_cmd_vel_.angular.z = raw_cmd_vel_.angular.z;
        judgeVel_pub_ptr->publish(output_cmd_vel_);
	}

}

void JudgesysRobot::PublishInfo()
{
    roboStatus_.max_hp = JudgesysControl::MaxHealth;
    roboStatus_.remain_hp = health_;
    roboStatus_.heat_cooling_limit = heat_cooling_limit_;
    roboStatus_.remain_ammo = ammo_;
    roboStatus_.chassis_output = !is_forbidden_to_move_;
    roboStatus_.shooter_output = !is_forbidden_to_shoot_;

    roboHeat_.shooter_heat = heat_;

    judgeStatus_pub_ptr->publish(roboStatus_);
    judgeHeat_pub_ptr->publish(roboHeat_);
}

void JudgesysRobot::Cool(int fre)
{
    if(health_ < 400)
        cooling_rate_ = JudgesysControl::CoolingRateDying;
    else {
        cooling_rate_ = JudgesysControl::CoolingRateNormal;
    }


    if(heat_ > JudgesysControl::HeatLimit)
    {
        if(heat_<360)
            health_ -= (heat_- JudgesysControl::HeatLimit)*40 / static_cast<float>(fre);
        else {
            health_ -= (heat_- 360)*400 / static_cast<float>(fre);
            heat_ = 360;
        }
        if(health_<0)
            health_ = 0;
    }

    heat_ -= cooling_rate_ / static_cast<float>(fre);
    if(heat_ < 0)
        heat_ = 0;

}

void JudgesysRobot::Shoot(int num, float speed)
{
    heat_ += num * speed;
    if(ammo_ > 0)
       ammo_ -- ;
    if(ammo_ <= 0)
        RCLCPP_ERROR(logger_,"[rm_judgesys] ammo is less than zero!");
}


void JudgesysRobot::DebuffMove()
{
    move_debuff_time_ = 10;
    RCLCPP_INFO(logger_,"[rm_judgesys]%s %s Debuff: Can't move!",color_.c_str(),robot_num_.c_str());
}

void JudgesysRobot::DebuffShoot()
{
    shoot_debuff_time_ = 10;
    RCLCPP_INFO(logger_,"[rm_judgesys]%s %s Debuff: Can't shoot!",color_.c_str(),robot_num_.c_str());
}

void JudgesysRobot::BuffAndDebuff(int fre)
{
    if(move_debuff_time_ > 0)
    {
        move_debuff_time_ -= 1.0f/fre;
        if(move_debuff_time_ < 0)
            move_debuff_time_ = 0;
        is_forbidden_to_move_ = true;
        //ROS_INFO("move_debuff_time_ = %f",move_debuff_time_);
    }
    else {
        is_forbidden_to_move_ = false;
    }


    if(shoot_debuff_time_ > 0)
    {
        shoot_debuff_time_ -= 1.0f/fre;
        if(shoot_debuff_time_ < 0)
            shoot_debuff_time_ = 0;
        is_forbidden_to_shoot_ = true;
    }
    else {
        is_forbidden_to_shoot_ = false;
    }
}

void JudgesysRobot::BeHit(rm_common::JudgeSysCommand command)
{
    switch (command) {
    case rm_common::JudgeSysCommand::ARMOR_HIT_FRONT:
        health_ -= 20;
        break;
    case rm_common::JudgeSysCommand::ARMOR_HIT_LEFT:
        health_ -= 40;
        break;
    case rm_common::JudgeSysCommand::ARMOR_HIT_BACK:
        health_ -=60;
        break;
    case rm_common::JudgeSysCommand::ARMOR_HIT_RIGHT:
        health_ -=40;
        break;
    default:
        RCLCPP_ERROR(logger_,"[rm_judgesys]Wrong armor hit command!");
        break;
    }
    if(health_<0)
        health_ = 0;
}

void JudgesysRobot::Reset()
{
    health_ = rm_judgesys::JudgesysControl::MaxHealth;
    heat_cooling_limit_ = rm_judgesys::JudgesysControl::HeatLimit;
    heat_ = 0;
    if(robot_num_ == "robot_0"|| robot_num_ == "robot_2")
        ammo_ = 50;
    else {
        ammo_ = 0;
    }
    move_debuff_time_ = 0;
    shoot_debuff_time_ = 0;
}

}


