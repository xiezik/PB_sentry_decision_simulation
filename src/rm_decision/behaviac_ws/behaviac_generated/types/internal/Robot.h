﻿// -------------------------------------------------------------------------------
// THIS FILE IS ORIGINALLY GENERATED BY THE DESIGNER.
// YOU ARE ONLY ALLOWED TO MODIFY CODE BETWEEN '///<<< BEGIN' AND '///<<< END'.
// PLEASE MODIFY AND REGENERETE IT IN THE DESIGNER FOR CLASS/MEMBERS/METHODS, ETC.
// -------------------------------------------------------------------------------

#ifndef _BEHAVIAC_HERO_DECISION_ROBOT_H_
#define _BEHAVIAC_HERO_DECISION_ROBOT_H_

#include "behaviac_headers.h"

///<<< BEGIN WRITING YOUR CODE FILE_INIT
#include "ros/ros.h"
#include "hero_msgs/RobotPosition.h"
#include "hero_msgs/BattlePosition.h"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/GetMap.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "hero_msgs/BasicExecutor.h"
#include "hero_msgs/RobotStatus.h"
#include "hero_msgs/RobotHeat.h"
#include "hero_msgs/Buffinfo.h"
#include "hero_msgs/GameStatus.h"
#include "costmap/costmap_interface.h"
#include "io/io.h"

//#include "global_planner_node.h"
///<<< END WRITING YOUR CODE

namespace hero_decision
{
///<<< BEGIN WRITING YOUR CODE NAMESPACE_INIT

///<<< END WRITING YOUR CODE

	class Robot : public behaviac::Agent
///<<< BEGIN WRITING YOUR CODE Robot
///<<< END WRITING YOUR CODE
	{
public:
    Robot();
    virtual ~Robot();

    BEHAVIAC_DECLARE_AGENTTYPE(hero_decision::Robot, behaviac::Agent)

    private: int ammo;

    private: int combat_effective_num;

    private: double combat_effectiveness;

    private: int health;

    private: behaviac::EBTStatus EngageRobot(behaviac::string robot_name);

    private: void Flee();

    private: behaviac::EBTStatus GoGetBuff();

    private: behaviac::EBTStatus WaitForGameStart();

///<<< BEGIN WRITING YOUR CODE CLASS_PART
public:
    void SetColor(std::string color);
    void ParaUpdate();

    typedef std::shared_ptr<hero_costmap::CostmapInterface> CostmapPtr;
    typedef std::shared_ptr<tf::TransformListener> TfPtr;

private:
    ros::NodeHandle nh_;
    std::string color_;
    std::string friendly_name_[2];
    std::string enemy_name_[2];
    hero_msgs::Buffinfo buffInfo_;
    hero_msgs::GameStatus gameStatus_;
    ros::Subscriber battle_position_sub_;
    ros::ServiceClient static_map_srv_;

    ros::Subscriber gameStatus_sub_;
    ros::Subscriber buffInfo_sub_;

    ros::ServiceClient basic_executor_cient_[2];

    nav_msgs::OccupancyGrid map_;
    hero_msgs::BattlePosition battle_position_;

    CostmapPtr costmap_ptr_;
    //！ Transform pointer
    TfPtr tf_ptr_;

    enum BuffType{
      FRIENDLY_HEAL = 0u,
      FRIENDLY_AMMO = 1u,
      ENEMY_HEAL = 2u,
      ENEMY_AMMO = 3u,
    };

    enum SideType{
      FRIENDLY_ONLY = 10000u,
      ENEMY_ONLY = 1001u,
      BOTH_SIDE= 1003u,
    };
    double RFID_F_x[6];
    double RFID_F_y[6];

    double RFID_height;
    double RFID_width;
    bool GetStaticMap();
    void GameStatusCallback(const hero_msgs::GameStatus::ConstPtr& msg);
    void BuffInfoCallback(const hero_msgs::Buffinfo::ConstPtr& msg);
    void BattlePositionCallback(const hero_msgs::BattlePosition::ConstPtr &msg);
    void MoveToPosition(int robot_num, double x,double y);
    void AttackRobot(int robot_num,int enemy_num);
    void GoGetBuff(int robot_num,int buff_num);
    void FleeBehaviour(int robot_num);
    int GetBuffRFIDNum(BuffType buff_type);
    void Init();
    void GetParam(ros::NodeHandle *nh);
    hero_msgs::RobotPosition FindRobotPosition(std::string robot_name);
    int FindClosestRobotToPosition(SideType specific_side,double x,double y);

///<<< END WRITING YOUR CODE
	};

///<<< BEGIN WRITING YOUR CODE NAMESPACE_UNINIT

///<<< END WRITING YOUR CODE
}

///<<< BEGIN WRITING YOUR CODE FILE_UNINIT

///<<< END WRITING YOUR CODE

#endif