#include "robot_physics.hpp"
#include <rclcpp/executors.hpp>
#include <string>
#include <sstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/utils.h>
#include "stdio.h"
#include "rm_decision_interfaces/msg/bullets_info.hpp"
#include <vector>
#include "../../rm_common/include/geometry.hpp"
#include "../../rm_common/include/math.hpp"
#include "../../rm_common/include/rm_map.hpp"
#include <thread>
#include <sys/time.h>
#include <cstdlib>// Header file needed to use rand
#include "std_msgs/msg/float32.hpp"


namespace rmMultistage {
RobotPhysics::RobotPhysics(const rclcpp::NodeOptions & options)
: Node("robot_physics_node", options){
    if (Init().IsOK()) {
        RCLCPP_INFO(this->get_logger(),"[robot physics]initialization completed.");
    } else {
        RCLCPP_ERROR(this->get_logger(),"[robot physics] Initialization failed.");
    }
}

rm_common::ErrorInfo RobotPhysics::Init() {
    for(int i =0;i<4;i++)
    need_shooting[i] = false;
    GetParam();

    try {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped = tfBuffer_->lookupTransform("map", "robot_0/odom", tf2::TimePointZero);
        AddRobot(new RobotTF("robot_0"));
        transformStamped = tfBuffer_->lookupTransform("map", "robot_1/odom", tf2::TimePointZero);
        AddRobot(new RobotTF("robot_1"));
        transformStamped = tfBuffer_->lookupTransform("map", "robot_2/odom", tf2::TimePointZero);
        AddRobot(new RobotTF("robot_2"));
        transformStamped = tfBuffer_->lookupTransform("map", "robot_3/odom", tf2::TimePointZero);
        AddRobot(new RobotTF("robot_3"));
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
    }
    
    pose_sub_[0] = this->create_subscription<nav_msgs::msg::Odometry>("/robot_0/base_pose_ground_truth", 
    rclcpp::SensorDataQoS(), 
    std::bind(&RobotPhysics::PoseCallback, this, std::placeholders::_1));
    pose_sub_[1] = this->create_subscription<nav_msgs::msg::Odometry>("/robot_1/base_pose_ground_truth", 
    rclcpp::SensorDataQoS(), 
    std::bind(&RobotPhysics::PoseCallback, this, std::placeholders::_1));
    pose_sub_[2] = this->create_subscription<nav_msgs::msg::Odometry>("/robot_2/base_pose_ground_truth", 
    rclcpp::SensorDataQoS(), 
    std::bind(&RobotPhysics::PoseCallback, this, std::placeholders::_1));
    pose_sub_[3] = this->create_subscription<nav_msgs::msg::Odometry>("/robot_3/base_pose_ground_truth", 
    rclcpp::SensorDataQoS(), 
    std::bind(&RobotPhysics::PoseCallback, this, std::placeholders::_1));

    roboHeat_ptr[0] = this->create_subscription<rm_decision_interfaces::msg::RobotHeat>("/judgeSysInfo/robot_0/heat_power", 
    rclcpp::SensorDataQoS(), 
    std::bind(&RobotPhysics::SetRobotHeat, this, std::placeholders::_1,0));
    roboHeat_ptr[1] = this->create_subscription<rm_decision_interfaces::msg::RobotHeat>("/judgeSysInfo/robot_1/heat_power", 
    rclcpp::SensorDataQoS(), 
    std::bind(&RobotPhysics::SetRobotHeat, this, std::placeholders::_1,1));
    roboHeat_ptr[2] = this->create_subscription<rm_decision_interfaces::msg::RobotHeat>("/judgeSysInfo/robot_2/heat_power", 
    rclcpp::SensorDataQoS(), 
    std::bind(&RobotPhysics::SetRobotHeat, this, std::placeholders::_1,2));
    roboHeat_ptr[3] = this->create_subscription<rm_decision_interfaces::msg::RobotHeat>("/judgeSysInfo/robot_3/heat_power", 
    rclcpp::SensorDataQoS(), 
    std::bind(&RobotPhysics::SetRobotHeat, this, std::placeholders::_1,3));

    bulletsInfo_pub_ = this->create_publisher<rm_decision_interfaces::msg::BulletsInfo>("robot_physic/bullet_info", 5);
    
    gimbalYaw_pub_[0] = this->create_publisher<std_msgs::msg::Float32MultiArray>("robot_0/gimbal_yaw_relative",5);
    gimbalYaw_pub_[1] = this->create_publisher<std_msgs::msg::Float32MultiArray>("robot_1/gimbal_yaw_relative",5);
    gimbalYaw_pub_[2] = this->create_publisher<std_msgs::msg::Float32MultiArray>("robot_2/gimbal_yaw_relative",5);
    gimbalYaw_pub_[3] = this->create_publisher<std_msgs::msg::Float32MultiArray>("robot_3/gimbal_yaw_relative",5);
    gimbalYaw_pub_[3] = this->create_publisher<std_msgs::msg::Float32MultiArray>("robot_3/gimbal_yaw_relative", 5);
    
    simu_decision_info_pub_ = this->create_publisher<rm_decision_interfaces::msg::BattlePosition>("simu_decision_info/battle_position",5);
    judgeSysClient_ = this->create_client<rm_decision_interfaces::srv::JudgeSysControl>("judgesys_control");
    static_map_srv_ = this->create_client<nav_msgs::srv::GetMap>("/static_map");

    // shoot_service_ = nh_.advertiseService("/shoot_server", &RobotPhysics::Shoot_handle_function,this);
    shoot_service_ = this->create_service<rm_decision_interfaces::srv::ShootCmd>(
    "shoot_server",
    [this](rm_decision_interfaces::srv::ShootCmd_Request::SharedPtr req,
           rm_decision_interfaces::srv::ShootCmd_Response::SharedPtr res) {
        return this->RobotPhysics::Shoot_handle_function(req, res);
    });
    
    shoot_hit_pub_ = this->create_publisher<rm_decision_interfaces::msg::JudgeSysShootHit>("judgeSysInfo/shoot_hit_event",5);
    
    
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client =
        this->create_client<nav_msgs::srv::GetMap>("/static_map");
    if (!client->wait_for_service(std::chrono::seconds(-1))) {
        RCLCPP_ERROR(this->get_logger(), "Failed to wait for service: /static_map");
    }
    nav_msgs::srv::GetMap::Request::SharedPtr req;
    auto result = static_map_srv_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(),"Received Static Map");
        map_ = result.get()->map;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),"Get static map failed");
    }

    return rm_common::ErrorInfo(rm_common::ErrorCode::OK);
}


void RobotPhysics::GetParam()
{
    this->declare_parameter<double>("RFID_F1_x", 7.63);
    this->get_parameter("RFID_F1_x", RFID_F_x[0]);

    this->declare_parameter<double>("RFID_F1_y", 1.8);
    this->get_parameter("RFID_F1_y", RFID_F_y[0]);

    this->declare_parameter<double>("RFID_F2_x", 6.23);
    this->get_parameter("RFID_F2_x", RFID_F_x[1]);

    this->declare_parameter<double>("RFID_F2_y", 3.225);
    this->get_parameter("RFID_F2_y", RFID_F_y[1]);

    this->declare_parameter<double>("RFID_F3_x", 4.03);
    this->get_parameter("RFID_F3_x", RFID_F_x[2]);

    this->declare_parameter<double>("RFID_F3_y", 0.49);
    this->get_parameter("RFID_F3_y", RFID_F_y[2]);

    this->declare_parameter<double>("RFID_F4_x", 0.45);
    this->get_parameter("RFID_F4_x", RFID_F_x[3]);

    this->declare_parameter<double>("RFID_F4_y", 3.34);
    this->get_parameter("RFID_F4_y", RFID_F_y[3]);

    this->declare_parameter<double>("RFID_F5_x", 1.85);
    this->get_parameter("RFID_F5_x", RFID_F_x[4]);

    this->declare_parameter<double>("RFID_F5_y", 1.915);
    this->get_parameter("RFID_F5_y", RFID_F_y[4]);

    this->declare_parameter<double>("RFID_F6_x", 4.05);
    this->get_parameter("RFID_F6_x", RFID_F_x[5]);

    this->declare_parameter<double>("RFID_F6_y", 4.9);
    this->get_parameter("RFID_F6_y", RFID_F_y[5]);

    this->declare_parameter<double>("RFID_height", 0.4);
    this->get_parameter("RFID_height", RFID_height);

    this->declare_parameter<double>("RFID_width", 0.46);
    this->get_parameter("RFID_width", RFID_width);
}

bool RobotPhysics::AddRobot(RobotTF *robot)
{
    if (FindRobot(robot->GetName()))
    {
        RCLCPP_ERROR(this->get_logger(),"[robot physics]%s already exist!",robot->GetName().c_str());
            return false;
    }
    robots_.push_back(robot);
    return true;
}

RobotTF* RobotPhysics::FindRobot(std::string robot_name)
{
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        if((*it)->GetName() == robot_name)
        {
            return (*it);
        }
    }
    return nullptr;
}

void RobotPhysics::PoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
    if(msg->header.frame_id.length() > 9) {
        std::string robot_name = ((std::string)(msg->header.frame_id)).substr(1,7);
        int i = robot_name.c_str()[6] - '0';
        if(i > 3) {
            RCLCPP_ERROR(this->get_logger(),"robot_%d? What the hell?", i);
            return;
        }
        tf2::Quaternion q;
        tf2::convert(msg->pose.pose.orientation, q);
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = rclcpp::Clock().now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = robot_name + "/base_pose_ground_truth";
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation = msg->pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(transformStamped);
    }
}


void RobotPhysics::PublishGimbalYaw()
{
    int i;
    for(i=0;i<4;i++)
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.push_back(static_cast<float>(robots_[i]->GetGimbalYaw()));
        gimbalYaw_pub_[i]->publish(msg);
    }

}



int RobotPhysics::ArmorHitDetect()
{
    int i;
    //static int time = 0;
    std::string robot_name[4] = {"robot_0","robot_1","robot_2","robot_3"};
    std::string aromor_name[4] = {"armor_front","armor_right","armor_back","armor_left"};
    std::vector<Point2D> pointsTest;
    int hitResult = 0;
    for(i=0;i<4;i++)//4 robots
    {

        if(robot_tf_received_[i])
        {

            robots_[i]->BulletHitDetect(Bullet("",0.0,0.0,0.0,0.0),&pointsTest);//even there is no bullet, we need run this function to get the coordinate of armor_plates(which will be published for the simu_decision)
            for(auto iter=bullets_.begin(); iter!=bullets_.end(); )
            {

            hitResult = robots_[i]->BulletHitDetect(**iter,&pointsTest);
            // ROS_INFO("hitResult = %d",hitResult);
                if(hitResult)
                {
                    if(hitResult > 0 && hitResult < 5)
                    {
                        RCLCPP_INFO(this->get_logger(),"%s hit %s %s",(*iter)->GetShooter().c_str(), robot_name[i].c_str(),aromor_name[hitResult - 1].c_str());
                        SendHitRobotInfo(robot_name[i],hitResult - 1);
                    }

                        iter = bullets_.erase(iter);
                }
                else
                        iter ++;
                }
    /*           for (auto it = pointsTest.begin(); it != pointsTest.end(); ++it) {
                bullets_.emplace_back(new Bullet("test",(*it).X(),(*it).Y(),0,0));
            }
*/
            }
        }

    //time++;
    //ROS_INFO("time:%d",time);

    return 0;
}

void RobotPhysics::BulletJudge()
{
    for(auto iter=bullets_.begin(); iter!=bullets_.end(); )
    {
        //hero_common::Polygon2D

            if( (*iter)->ReachBoundary(8,8)||
                    rm_common::LineSegmentIsIntersectMapObstacle(&map_,(*iter)->GetPositionNow().X(),(*iter)->GetPositionNow().Y(),
                                                                (*iter)->GetPositionLast().X(),(*iter)->GetPositionLast().Y(),50))
                iter = bullets_.erase(iter);
            else
                iter ++ ;
    }
}


void RobotPhysics::PublishBulletsInfo()
{
    rm_decision_interfaces::msg::BulletsInfo bulletInfo;
    bulletInfo.bullet_num = bullets_.size();
    for(size_t i = 0; i< bullets_.size();i++)
    {
        bulletInfo.bullets.emplace_back(bullets_.at(i)->GetBulletMove());
    }
    bulletsInfo_pub_->publish(bulletInfo);

}

void RobotPhysics::PublishSimuDecisionInfo()
{
    rm_decision_interfaces::msg::RobotPosition robot_position;
    rm_decision_interfaces::msg::BattlePosition battle_position;
    double roll,pitch;

    battle_position.robots_num = 0;
    battle_position.robots_position.clear();
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
    battle_position.robots_num ++ ;
    robot_position.robot_name = (*it)->GetName();
    robot_position.position.x = (*it)->robot_tf.getOrigin().getX();
    robot_position.position.y = (*it)->robot_tf.getOrigin().getY();
    tf2::Matrix3x3((*it)->robot_tf.getRotation()).getRPY(roll, pitch, robot_position.position.yaw);
    for(int i =0;i<4;i++)
    {
        robot_position.armor_plates[i].x = (*it)->GetChassisSidesMid(i).X();
        robot_position.armor_plates[i].y = (*it)->GetChassisSidesMid(i).Y();
        robot_position.armor_plates[i].yaw = robot_position.position.yaw + rm_common::PI * 0.5 * i;
        robot_position.ammo = FindRobot((*it)->GetName())->roboStatus_.remain_ammo;
        if(robot_position.armor_plates[i].yaw>rm_common::PI)
        robot_position.armor_plates[i].yaw -= 2*rm_common::PI;
        if(robot_position.armor_plates[i].yaw<-rm_common::PI)
        robot_position.armor_plates[i].yaw += 2*rm_common::PI;

    }
    robot_position.health = (*it)->roboStatus_.remain_hp;
    battle_position.robots_position.emplace_back(robot_position);
    }
    simu_decision_info_pub_->publish(battle_position);
}

void RobotPhysics::RobotShoot(std::string robot_name)
{
    RobotTF *robotf = FindRobot(robot_name);
    float yaw_distribute = ((rand()%2000)/1000.0f - 1) * DistributeYaw;
    float speed_distribute = ((rand()%2000)/1000.0f - 1) * DistributeSpeed;
    if(robot_name=="robot_0")
    {
        if(FindRobot(robot_name)->roboStatus_.remain_ammo <= 0||
        FindRobot(robot_name)->roboStatus_.remain_hp<=0 ||
         !FindRobot(robot_name)->roboStatus_.shooter_output||
         robotHeat[0]->shooter_heat > HeatControlLimit)
            return;
    }
    else if(robot_name=="robot_1")
    {
        if(FindRobot(robot_name)->roboStatus_.remain_ammo <= 0 ||
         FindRobot(robot_name)->roboStatus_.remain_hp<=0 ||
          !FindRobot(robot_name)->roboStatus_.shooter_output||
          robotHeat[1]->shooter_heat > HeatControlLimit)
            return;
    }
    else if(robot_name=="robot_2")
    {
        if(FindRobot(robot_name)->roboStatus_.remain_ammo <= 0||
        FindRobot(robot_name)->roboStatus_.remain_hp<=0 ||
         !FindRobot(robot_name)->roboStatus_.shooter_output ||
         robotHeat[2]->shooter_heat > HeatControlLimit)
            return;
    }
    else if(robot_name=="robot_3")
    {
        if(FindRobot(robot_name)->roboStatus_.remain_ammo <= 0||
        FindRobot(robot_name)->roboStatus_.remain_hp<=0 ||
         !FindRobot(robot_name)->roboStatus_.shooter_output||
         robotHeat[3]->shooter_heat > HeatControlLimit)
            return;
    }
    else
        return;

    if(robotf)
    {

        SendShootRobotInfo(robot_name);
        bullets_.emplace_back(new Bullet(robot_name,robotf->robot_tf.getOrigin().getX(),robotf->robot_tf.getOrigin().getY(),robotf->GetGimbalAbsoluteYaw() + yaw_distribute,23+speed_distribute));
    }
}

void RobotPhysics::LetBulletsFly(int fre)
{
    size_t i;
    for(i = 0; i < bullets_.size(); i++)
    {
        bullets_.at(i)->Move(fre);
    }
}

void RobotPhysics::SendHitRobotInfo(std::string robot_name, int armor_num)
{
    //SendJudgeSysRequest(robot_name,hero_common::JudgeSysCommand::ARMOR_HIT_FRONT + armor_num);
    rm_decision_interfaces::msg::JudgeSysShootHit shoot_hit_info;
    shoot_hit_info.robot_name = robot_name;
    shoot_hit_info.command = rm_common::JudgeSysCommand::ARMOR_HIT_FRONT + armor_num;
    shoot_hit_pub_->publish(shoot_hit_info);
}

void RobotPhysics::SendShootRobotInfo(std::string robot_name)
{
    //SendJudgeSysRequest(robot_name,hero_common::JudgeSysCommand::SHOOT_BULLET);
    rm_decision_interfaces::msg::JudgeSysShootHit shoot_hit_info;
    shoot_hit_info.robot_name = robot_name;
    shoot_hit_info.command = rm_common::JudgeSysCommand::SHOOT_BULLET;
    shoot_hit_pub_->publish(shoot_hit_info);
}


void RobotPhysics::SetRobotHeat(const rm_decision_interfaces::msg::RobotHeat::SharedPtr& msg, int index)
{
    robotHeat[index] = msg;
}

void RobotPhysics::RFID_detect()
{
    int i,j;
    static int infoDivider;

    for(i = 0;i<4;i++)//robot
    {
        for(j = 0;j<6;j++)//RFID
        {
            if(rm_common::PointInRect(robots_[i]->robot_tf.getOrigin().getX(),robots_[i]->robot_tf.getOrigin().getY(),RFID_F_x[j],RFID_F_y[j],
                                        RFID_F_x[j] + RFID_width ,RFID_F_y[j] + RFID_height))
            {
                //ROS_INFO("robot: %f,%f",robots_[i]->robot_tf.getOrigin().getX(),robots_[i]->robot_tf.getOrigin().getY());
                //ROS_INFO("rect %d: %f,%f, %f,%f",j+1,RFID_F_x[j],RFID_F_y[j],RFID_F_x[j] + RFID_width ,RFID_F_y[j] + RFID_height);
                infoDivider ++;
                if(infoDivider % 20 == 0)
                        RCLCPP_INFO(this->get_logger(),"%s just pass RFID %d.",robots_[i]->GetName().c_str(),j + 1);
                SendJudgeSysRequest(robots_[i]->GetName(),rm_common::JudgeSysCommand::RFID_F1 + j);
                    //bullets_.emplace_back(new Bullet("robot",RFID_F_x[j],RFID_F_y[j],0,5));
            }
            else
                infoDivider = 0;
            // ROS_INFO("robot: %f,%f",robots_[i]->robot_tf.getOrigin().getX(),robots_[i]->robot_tf.getOrigin().getY());
            // ROS_INFO("rect %d: %f,%f, %f,%f",j+1,RFID_F_x[j],RFID_F_y[j],RFID_F_x[j] + RFID_width ,RFID_F_y[j] + RFID_height);
        }
    }
}

void RobotPhysics::SendJudgeSysRequest(std::string robot_name, int command)
{
    auto srv = std::make_shared<rm_decision_interfaces::srv::JudgeSysControl::Request>();
    srv->command = command;
    srv->robot_name = robot_name;
    auto future = judgeSysClient_->async_send_request(srv);
    rclcpp::spin_until_future_complete(judgeSysClient_, future); // Fix for problem 1
    if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) // Fix for problem 2
    {
        auto response = future.get();
        /*if(response->error_code!=0)
        {
            RCLCPP_ERROR(node_->get_logger(), "judgesys service error: %d", response->error_code);
        }*/
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call judgesys service");
    }
}

bool RobotPhysics::Shoot_handle_function(rm_decision_interfaces::srv::ShootCmd_Request::SharedPtr &req,
rm_decision_interfaces::srv::ShootCmd_Response::SharedPtr &res)
{
    res->received = true;
    if(req->mode == req->ONCE)
    {
        if(req->robot_num=="robot_0")
        need_shooting[0] = true;
        if(req->robot_num=="robot_1")
        need_shooting[1] = true;
        if(req->robot_num=="robot_2")
        need_shooting[2] = true;
        if(req->robot_num=="robot_3")
        need_shooting[3] = true;
        // RobotShoot(req.robot_num);
    }
    //ROS_INFO("shoot request receive!");
    return true;
}

void RobotPhysics::GimbalsMove()
{
    for (auto it = robots_.begin(); it != robots_.end(); ++it) {
        (*it)->GimbalMove();
    }
}

void RobotPhysics::ShootSequence()
{
    static int divider = 0;
    if(divider<8)
    {
        divider ++;
        return;
    }
    divider = 0;
    if(need_shooting[0])
        RobotShoot("robot_0");
    if(need_shooting[1])
        RobotShoot("robot_1");
    if(need_shooting[2])
        RobotShoot("robot_2");
    if(need_shooting[3])
        RobotShoot("robot_3");
    for(int i =0;i<4;i++)
        need_shooting[i] = false;
}
}



void ROS_Spin()
{
    rclcpp::spin(std::make_shared<rmMultistage::RobotPhysics>());
}

int main(int argc, char** argv){
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create NodeOptions object
    rclcpp::NodeOptions options;

    // Use NodeOptions object to create RobotPhysics object
    auto robotPhysics = std::make_shared<rmMultistage::RobotPhysics>(options);

    // Create a thread to spin the ROS 2 node
    std::thread spin_thread(ROS_Spin);
int ms = 0;
int ms_last = 0;
struct timeval tv;
struct timezone tz;

while (rclcpp::ok()) {
    gettimeofday(&tv, &tz);
    ms_last = tv.tv_sec * 1000 + tv.tv_usec / 1000;

    robotPhysics->GimbalsMove();
    robotPhysics->PublishGimbalYaw();
    robotPhysics->LetBulletsFly(50);
    robotPhysics->PublishBulletsInfo();
    robotPhysics->BulletJudge();
    robotPhysics->ArmorHitDetect();
    robotPhysics->RFID_detect();
    robotPhysics->PublishSimuDecisionInfo();
    robotPhysics->ShootSequence();

    gettimeofday(&tv, &tz);
    ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    if (ms - ms_last < 20) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20 - (ms - ms_last)));
    }
}

rclcpp::shutdown();
return 0;
}


