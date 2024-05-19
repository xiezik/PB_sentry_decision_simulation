
#include "rm_judgesys/judgesys_control_node.hpp"
#include <rm_decision_interfaces/srv/detail/judge_sys_control__struct.hpp>
#include <string>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <cstdlib>// Header file needed to use rand
#include <random> // Include the <random> header for random_device and default_random_engine.

namespace rm_judgesys{

JudgesysControl::JudgesysControl(const rclcpp::NodeOptions & options)
: Node("judgesys_control", options){
   if (Init().IsOK()) {
    RCLCPP_INFO(this->get_logger(),"judgesys control initialization completed.");
  } else {
    RCLCPP_ERROR(this->get_logger(),"judgesys control Initialization failed.");
  }
}

JudgesysRobot* JudgesysControl::FindRobot(std::string robot_name)
{
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        if((*it)->GetName() == robot_name)
        {
            return (*it);
        }
    }
    return nullptr;
}

void JudgesysControl::ResetAllRobot()
{
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        (*it)->Reset();

    }
}

void JudgesysControl::SetGamePhase(int phase)
{
    if(phase == rm_common::JudgeSysCommand::GAME_PERP)
    {
        gamePhase_ = GamePhase::PREPERATION;
        game_time = 180;
        RCLCPP_INFO(this->get_logger(),"[rm_judgesys]Set game preperation");
    }
    else if(phase == rm_common::JudgeSysCommand::GAME_START)
    {
        gamePhase_ = GamePhase::START;
        game_time = 180;
        RCLCPP_INFO(this->get_logger(),"[rm_judgesys]Set game start");
        //RFID_Refresh();
        ResetAllRobot();
    }
    else if(phase == rm_common::JudgeSysCommand::GAME_END)
    {
        gamePhase_ = GamePhase::END;
        game_time = 0;
        RCLCPP_INFO(this->get_logger(),"[rm_judgesys]Set game end");
    }

}

void JudgesysControl::RFID_Callback(std::string robot_name, int num)
{
    if(RFID[num].isActivated||FindRobot(robot_name)->GetHealth()==0)
        return;
    if(RFID[num].color == "red")
    {
        if(RFID[num].type == RFID_type::HEAL)
        {
            FindRobot("robot_2")->BuffHeal();
            FindRobot("robot_3")->BuffHeal();
        }
        else if(RFID[num].type == RFID_type::RELOAD)
        {
            FindRobot("robot_2")->BuffReload();
            FindRobot("robot_3")->BuffReload();
        }
    }
    else if(RFID[num].color == "blue")
    {
        if(RFID[num].type == RFID_type::HEAL)
        {
            FindRobot("robot_0")->BuffHeal();
            FindRobot("robot_1")->BuffHeal();
        }
        else if(RFID[num].type == RFID_type::RELOAD)
        {
            FindRobot("robot_0")->BuffReload();
            FindRobot("robot_1")->BuffReload();
        }
    }
    else {
        if(RFID[num].type == RFID_type::MOVE_DEFBUFF)
            FindRobot(robot_name)->DebuffMove();
        else if(RFID[num].type == RFID_type::SHOOT_DEBUFF)
            FindRobot(robot_name)->DebuffShoot();
    }
    RFID[num].isActivated = true;
}
void JudgesysControl::HitRobot(std::string robot_name, rm_common::JudgeSysCommand command)
{
    FindRobot(robot_name)->BeHit(command);
}

void JudgesysControl::RobotShoot(std::string robot_name)
{
    FindRobot(robot_name)->Shoot(1,22);
}
rm_common::ErrorInfo JudgesysControl::Init() {
    AddRobot( new JudgesysRobot("robot_0","blue"));
    AddRobot( new JudgesysRobot("robot_1","blue"));
    AddRobot( new JudgesysRobot("robot_2","red"));
    AddRobot( new JudgesysRobot("robot_3","red"));

    service_ = this->create_service<rm_decision_interfaces::srv::JudgeSysControl>(
    "judgesys_control",
    [this](rm_decision_interfaces::srv::JudgeSysControl_Request::SharedPtr req,
           rm_decision_interfaces::srv::JudgeSysControl_Response::SharedPtr res) {
        return this->handle_function(req, res);
    });
    
    gameState_pub_ = this->create_publisher<rm_decision_interfaces::msg::GameStatus>("judgeSysInfo/game_state", 5);
    buffInfo_pub_ = this->create_publisher<rm_decision_interfaces::msg::Buffinfo>("judgeSysInfo/buff_info", 5);
    
    shoot_hit_sub_ = this->create_subscription<rm_decision_interfaces::msg::JudgeSysShootHit>("judgeSysInfo/shoot_hit_event", 
    rclcpp::SensorDataQoS(), 
    std::bind(&JudgesysControl::ShootHitCallback, this, std::placeholders::_1));
    
    for(int i=0;i<6;i++)
        buffZone[i] = false;
    game_time = 0;
    gamePhase_ = GamePhase::PREPERATION;

	//KillRobot("robot_1");
	//KillRobot("robot_2");
	//KillRobot("robot_3");
    RFID_Refresh();
	return rm_common::ErrorInfo(rm_common::ErrorCode::OK);
}

void JudgesysControl::ShootHitCallback(const rm_decision_interfaces::msg::JudgeSysShootHit::ConstSharedPtr &msg)
{
  std::string  robot_name = msg->robot_name;
  switch (msg->command) {
  case rm_common::JudgeSysCommand::ARMOR_HIT_FRONT:
    HitRobot(msg->robot_name,rm_common::JudgeSysCommand::ARMOR_HIT_FRONT);
    RCLCPP_INFO(this->get_logger(),"[rm_judgesys] %s get hit on front armor", robot_name.c_str());
    break;
  case rm_common::JudgeSysCommand::ARMOR_HIT_LEFT:
    HitRobot(msg->robot_name,rm_common::JudgeSysCommand::ARMOR_HIT_RIGHT);
    RCLCPP_INFO(this->get_logger(),"[rm_judgesys] %s get hit on right armor",robot_name.c_str());
    break;
  case rm_common::JudgeSysCommand::SHOOT_BULLET:
    RobotShoot(msg->robot_name);
    break;
  }
}

bool JudgesysControl::handle_function(rm_decision_interfaces::srv::JudgeSysControl_Request::SharedPtr &req,
                    rm_decision_interfaces::srv::JudgeSysControl_Response::SharedPtr &res)
{
        res->error_code = 0;
        std::string  robot_name = req->robot_name;
    switch (req->command) {
    case rm_common::JudgeSysCommand::KILL_ROBOT:
        KillRobot(req->robot_name);
        break;
    case rm_common::JudgeSysCommand::REVIVE_ROBOT:
        ReviveRobot(req->robot_name);
        break;
    case rm_common::JudgeSysCommand::RELOAD_ROBOT:
        ReloadRobot(req->robot_name);
        break;
    case rm_common::JudgeSysCommand::DISARM_ROBOT:
        DisarmRobot(req->robot_name);
        break;
    case rm_common::JudgeSysCommand::ARMOR_HIT_FRONT:
        HitRobot(req->robot_name,rm_common::JudgeSysCommand::ARMOR_HIT_FRONT);
        RCLCPP_INFO(this->get_logger(),"[rm_judgesys] %s get hit on front armor", robot_name.c_str());
        break;
    case rm_common::JudgeSysCommand::ARMOR_HIT_LEFT:
        HitRobot(req->robot_name,rm_common::JudgeSysCommand::ARMOR_HIT_LEFT);
        RCLCPP_INFO(this->get_logger(),"[rm_judgesys] %s get hit on left armor",robot_name.c_str());
        break;
    case rm_common::JudgeSysCommand::ARMOR_HIT_BACK:
        HitRobot(req->robot_name,rm_common::JudgeSysCommand::ARMOR_HIT_BACK);
        RCLCPP_INFO(this->get_logger(),"[rm_judgesys] %s get hit on back armor, that's hurt!",robot_name.c_str());
        break;
    case rm_common::JudgeSysCommand::ARMOR_HIT_RIGHT:
        HitRobot(req->robot_name,rm_common::JudgeSysCommand::ARMOR_HIT_RIGHT);
        RCLCPP_INFO(this->get_logger(),"[rm_judgesys] %s get hit on right armor",robot_name.c_str());
        break;
    case rm_common::JudgeSysCommand::SHOOT_BULLET:
        RobotShoot(req->robot_name);
        break;
    case rm_common::JudgeSysCommand::RFID_F1:
    case rm_common::JudgeSysCommand::RFID_F2:
    case rm_common::JudgeSysCommand::RFID_F3:
    case rm_common::JudgeSysCommand::RFID_F4:
    case rm_common::JudgeSysCommand::RFID_F5:
    case rm_common::JudgeSysCommand::RFID_F6:
        RFID_Callback(req->robot_name,req->command - rm_common::JudgeSysCommand::RFID_F1);
        break;
    case rm_common::JudgeSysCommand::REFRESH_RFID:
        RFID_Refresh();
        break;
    case rm_common::JudgeSysCommand::KILL_ALL:
        KillRobot("all");
        RCLCPP_INFO(this->get_logger(),"[rm_judgesys]Kill all");
        break;
    case rm_common::JudgeSysCommand::GAME_PERP:
    case rm_common::JudgeSysCommand::GAME_START:
    case rm_common::JudgeSysCommand::GAME_END:
        SetGamePhase(req->command);
        break;
    default:   res->error_code = 1;
        break;
    }
//	RCLCPP_INFO(this->get_logger(),"Request from %s with age %d ", req->name.c_str(), req->age);

  // 返回一个反馈，将response设置为"..."

  return true;
}


void JudgesysControl::GameTick(int fre)
{
    static double time_last;
    if(gamePhase_ == GamePhase::START)
    {
         game_time -= 1.0 / fre;

         if((time_last > 60 && game_time<60)||((time_last > 120 && game_time<120)))
             RFID_Refresh();

         if(game_time < 0)
         {
             game_time = 0;
             gamePhase_ = GamePhase::END;
         }
         time_last = game_time;
    }
    else  if (gamePhase_ == GamePhase::PREPERATION)
        game_time = 180;
    else {
        game_time = 0;
    }

}

void JudgesysControl::UpdateVel()
{
	for (auto it = robots_.begin(); it != robots_.end(); ++it) {
		(*it)->PublishVel();
	}
}

void JudgesysControl::UpdateInfo()
{
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        (*it)->PublishInfo();
    }
    rm_decision_interfaces::msg::GameStatus gameState;
    if(gamePhase_ == GamePhase::PREPERATION)
        gameState.game_status = gameState.PRE_MATCH;
    else if(gamePhase_ == GamePhase::START)
    {
        gameState.game_status = gameState.ROUND;
    }
    else if(gamePhase_ == GamePhase::END)
        gameState.game_status = gameState.CALCULATION;
    gameState.remaining_time = (int)game_time;
    gameState_pub_->publish(gameState);

    rm_decision_interfaces::msg::Buffinfo buffInfo;
    for(int i=0;i<6;i++)
    {
        if(RFID[i].type == RFID_type::HEAL)
        {
            if(RFID[i].color=="red")
                buffInfo.buff_data.emplace_back(buffInfo.RED_HEAL);
            else if(RFID[i].color=="blue")
                buffInfo.buff_data.emplace_back(buffInfo.BLUE_HEAL);
        }
        else if(RFID[i].type == RFID_type::RELOAD)
        {
            if(RFID[i].color=="red")
                buffInfo.buff_data.emplace_back(buffInfo.RED_RELOAD);
            else if(RFID[i].color=="blue")
                buffInfo.buff_data.emplace_back(buffInfo.BLUE_RELOAD);
        }
        else if(RFID[i].type == RFID_type::MOVE_DEFBUFF)
            buffInfo.buff_data.emplace_back(buffInfo.MOVE_DEBUFF);
        else if(RFID[i].type == RFID_type::SHOOT_DEBUFF)
            buffInfo.buff_data.emplace_back(buffInfo.SHOOT_DEBUFF);
        buffInfo.activated.emplace_back(RFID[i].isActivated);

    }

    buffInfo_pub_->publish(buffInfo);
}

bool JudgesysControl::KillRobot(std::string robot_num)
{
	bool val = false;
	 for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        if((*it)->GetName() == robot_num||robot_num=="all")
        {
            (*it)->Kill();
                        RCLCPP_INFO(this->get_logger(),"[rm_judgesys]Kill %s\n",(*it)->GetName().c_str());
						val = true;
        }
    }
		return val;
}

bool JudgesysControl::ReviveRobot(std::string robot_num)
{
    bool val = false;
     for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        if((*it)->GetName() == robot_num)
        {
            (*it)->Revive();
                        RCLCPP_INFO(this->get_logger(),"[rm_judgesys]Revive %s\n",(*it)->GetName().c_str());
                        val = true;
        }
    }
    return val;
}

bool JudgesysControl::ReloadRobot(std::string robot_num)
{
    bool val = false;
     for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        if((*it)->GetName() == robot_num)
        {
            (*it)->Reload();
                        RCLCPP_INFO(this->get_logger(),"[rm_judgesys]Reload %s ammo = %d\n",(*it)->GetName().c_str(),(*it)->GetAmmo());
                        val = true;
        }
    }
    return val;
}

bool JudgesysControl::DisarmRobot(std::string robot_num)
{
    bool val = false;
     for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        if((*it)->GetName() == robot_num)
        {
            (*it)->Disarm();
                        RCLCPP_INFO(this->get_logger(),"[rm_judgesys]Disarm %s\n!",(*it)->GetName().c_str());
                        val = true;
        }
    }
    return val;
}

void JudgesysControl::RobotsCooling(int fre)
{
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
     (*it)->Cool(fre);
   }

}

void JudgesysControl::RFID_Refresh()
{
    std::vector<RFID_type> rfid_vector;
    rfid_vector.clear();
    rfid_vector.emplace_back(RFID_type::HEAL);
    
    std::string color[2] = {"red","blue"};
    std::string type[4] = {"heal","reload","move_debuff","shoot_debuff"};
    int k;

    std::shuffle(rfid_vector.begin(), rfid_vector.end(), std::default_random_engine(std::random_device()()));
    for(int i =0;i<3;i++)
    {
        if(rfid_vector[i] != RFID_type::MOVE_DEFBUFF && rfid_vector[i] != RFID_type::SHOOT_DEBUFF)
        {
            k = rand()%2;
            RFID[i].color = color[k];
            RFID[i].type = rfid_vector[i];
            RFID[i + 3].color = color[1 - k];
            RFID[i + 3].type = rfid_vector[i];
        }
        else {
            if(rand()%2){
                RFID[i].type = RFID_type::MOVE_DEFBUFF;
                RFID[i + 3].type = RFID_type::SHOOT_DEBUFF;
                RFID[i].color = "both";
                RFID[i + 3].color = "both";
            }
            else {
                RFID[i].type = RFID_type::SHOOT_DEBUFF;
                RFID[i + 3].type = RFID_type::MOVE_DEFBUFF;
                RFID[i].color = "both";
                RFID[i + 3].color = "both";
            }
        }
    }

    RCLCPP_INFO(this->get_logger(),"[rm_judgesys]RFID refresh!");
    for(int i =0;i<6;i++)
    {
      RFID[i].isActivated = false;
      RCLCPP_INFO(this->get_logger(),"[rm_judgesys]RFID F%d:%s %s",i+1, RFID[i].color.c_str(),type[RFID[i].type - RFID_type::HEAL].c_str());
    }
}

void JudgesysControl::LogOutput()
{
    static int robot_health[4] = {2000,2000,2000,2000};
    for(int i = 0;i < 4;i++)
    {
        if(robot_health[i] > 0 && robots_[i]->GetHealth() <=0)
            RCLCPP_INFO(this->get_logger(),"[rm_judgesys]%s is dead!",robots_[i]->GetName().c_str());
        robot_health[i] = robots_[i]->GetHealth();

    }
}

void JudgesysControl::BuffHandle(int fre)
{
    for(int i = 0;i < 4;i++)
    {
        robots_[i]->BuffAndDebuff(fre);

    }
}

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 创建 NodeOptions 对象
    rclcpp::NodeOptions options;

    // 使用 NodeOptions 对象创建 JudgesysControl 对象
    auto judgesys_control = std::make_shared<rm_judgesys::JudgesysControl>(options);

    int ms = 0;
    int ms_last = 0;
    struct timeval tv;
    struct timezone tz;

    while (rclcpp::ok()) {
        gettimeofday(&tv, &tz);
        ms_last = tv.tv_sec * 1000 + tv.tv_usec / 1000;

        rclcpp::spin_some(judgesys_control);
        judgesys_control->UpdateVel();
        judgesys_control->UpdateInfo();
        judgesys_control->RobotsCooling(40);
        judgesys_control->BuffHandle(40);
        judgesys_control->LogOutput();
        judgesys_control->GameTick(40);

        gettimeofday(&tv, &tz);
        ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
        if (ms - ms_last < 25) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25 - (ms - ms_last)));
        }
    }

    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_judgesys::JudgesysControl)