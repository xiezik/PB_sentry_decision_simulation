#include "robot_tf.hpp"
#include <rclcpp/clock.hpp>
#include <rm_decision_interfaces/srv/detail/gimbal_aim__struct.hpp>
#include <rm_decision_interfaces/srv/gimbal_aim.hpp>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace rmMultistage {

    using rm_common::ErrorCode;
    using rm_common::ErrorInfo;
    using rm_common::Point2D;
    using rm_common::Polygon2D;
    using rm_common::LineSegment2D;

    RobotTF::RobotTF(std::string robot_name):Node("robot_tf"),
      gimbal_yaw_(0),
      gimbal_yaw_set_(0)
    {
        if (Init(robot_name).IsOK()) {
         ROS_INFO("[robot_physicis]robot %s added.",robot_name.c_str());
       } else {
         ROS_ERROR("[robot_physicis]fail to add robot: %s.",robot_name.c_str());
       }
    }

    ErrorInfo RobotTF::Init(std::string robot_name) {
        
        robot_name_ = robot_name;
        gimbal_aim_service_ = this->create_service<rm_decision_interfaces::srv::GimbalAim>(
        "/"+robot_name_+"/gimbal_aim_server",
        [this](rm_decision_interfaces::srv::GimbalAim_Request::SharedPtr req,
              rm_decision_interfaces::srv::GimbalAim_Response::SharedPtr res) {
            return this->RobotTF::GimbalAim_handle_function(req, res);
        });
        judgeStatus_sub_ = this->create_subscription<rm_decision_interfaces::msg::RobotStatus>(
        "/judgeSysInfo/"+robot_name_+"/status", 
        rclcpp::SensorDataQoS(), 
        std::bind(&RobotTF::RobotStatusCallback, this, std::placeholders::_1));
        
        return ErrorInfo(ErrorCode::OK);
    }

    void RobotTF::RobotStatusCallback(const rm_decision_interfaces::msg::RobotStatus::ConstSharedPtr &msg)
    {
      roboStatus_ = *msg;
    }


    bool RobotTF::GimbalAim_handle_function(rm_decision_interfaces::srv::GimbalAim_Request::SharedPtr &req, rm_decision_interfaces::srv::GimbalAim_Response::SharedPtr &res)
    {
      SetGimbalAbsoluteYaw(req->set_angle_absolute);
      res->aimed = (std::abs(rm_common::GetAngleInRange(req->set_angle_absolute - GetGimbalAbsoluteYaw()))<0.02);
      //ROS_INFO("[%s]set %f ,now %f, err = %f",robot_name_.c_str(), req.set_angle_absolute , GetGimbalAbsoluteYaw(),std::abs(hero_common::GetAngleInRange(req.set_angle_absolute - GetGimbalAbsoluteYaw())));
      return true;
    }


    float RobotTF::GetGimbalAbsoluteYaw()
    {
        double roll,pitch,yaw;
        tf2::Matrix3x3(robot_tf.getRotation()).getRPY(roll, pitch, yaw);
        gimbal_yaw_absolute_ = (yaw + gimbal_yaw_);
        return rm_common::GetAngleInRange(gimbal_yaw_absolute_);

    }

    void RobotTF::SetGimbalAbsoluteYaw(double set_yaw_absolute)
    {
      double roll,pitch,yaw;
      double set_yaw;
      tf2::Matrix3x3(robot_tf.getRotation()).getRPY(roll, pitch, yaw);
      set_yaw = set_yaw_absolute - yaw;
      SetGimbalEncoderYaw(rm_common::GetAngleInRange(set_yaw));
    }

    bool RobotTF::SetGimbalEncoderYaw(double yaw_set)
    {
      if(yaw_set>rm_common::PI*0.5)
        yaw_set = rm_common::PI*0.5;
      if(yaw_set<-rm_common::PI*0.5)
        yaw_set = -rm_common::PI*0.5; // machanical limit
      gimbal_yaw_set_ = yaw_set;
    }

    bool RobotTF::GimbalMove()
    {
      if(roboStatus_.remain_hp==0)
        return false;
      if(gimbal_yaw_set_ - gimbal_yaw_ > GimbalMaxSpeed)
      {
          gimbal_yaw_ += GimbalMaxSpeed;
          return false;
      }
      else if(gimbal_yaw_set_ - gimbal_yaw_ > 0){
        gimbal_yaw_ = gimbal_yaw_set_;
        return true;
      }
      if(gimbal_yaw_set_ - gimbal_yaw_ <- GimbalMaxSpeed)
      {
          gimbal_yaw_ -= GimbalMaxSpeed;
          return false;
      }
      else if(gimbal_yaw_set_ - gimbal_yaw_ < 0){
        gimbal_yaw_ = gimbal_yaw_set_;
        return true;
      }
      return false;
    }

    void RobotTF::PublishArmorTF()
{
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;

    transformStamped.header.frame_id = robot_name_ + "/base_pose_ground_truth";
    transformStamped.header.stamp = ros::Time::now();

    // armor_front
    transformStamped.child_frame_id = robot_name_ + "/armor_front";
    transformStamped.transform.translation.x = RobotWidth / 2;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    broadcaster_->sendTransform(transformStamped);

    // armor_back
    transformStamped.child_frame_id = robot_name_ + "/armor_back";
    transformStamped.transform.translation.x = -RobotWidth / 2;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    q.setRPY(0, 0, rm_common::PI);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    broadcaster_->sendTransform(transformStamped);

    // armor_left
    transformStamped.child_frame_id = robot_name_ + "/armor_left";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = RobotLength / 2;
    transformStamped.transform.translation.z = 0;
    q.setRPY(0, 0, rm_common::PI * 0.5);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    broadcaster_->sendTransform(transformStamped);

    // armor_right
    transformStamped.child_frame_id = robot_name_ + "/armor_right";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = -RobotLength / 2;
    transformStamped.transform.translation.z = 0;
    q.setRPY(0, 0, -rm_common::PI * 0.5);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    broadcaster_->sendTransform(transformStamped);

    // gimbal
    transformStamped.child_frame_id = robot_name_ + "/gimbal";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    q.setRPY(0, 0, gimbal_yaw_);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    broadcaster_->sendTransform(transformStamped);
}


    int RobotTF::BulletHitDetect(Bullet bullet,std::vector<Point2D> *pointsfeedback)
    {
        double roll,pitch,yaw;
        int i;
        tf2::Matrix3x3(robot_tf.getRotation()).getRPY(roll, pitch, yaw);
        Point2D chassisMid(robot_tf.getOrigin().getX(),robot_tf.getOrigin().getY());
        double chassisYaw = yaw;
        LineSegment2D bulletSeg(bullet.GetPositionNow(),bullet.GetPositionLast());
        Point2D interSectionPoint;
        std::vector<Point2D> pointsIntersect;
        std::vector<int> sides;
        Point2D chassisLeft;
        Point2D chassisRight;
        pointsfeedback->clear();
        pointsfeedback->emplace_back(chassisMid);
        bool isHit = false;
        if(bullet.GetShooter() == robot_name_)
            return 0;
        for(i=0;i<4;i++) //front left back right
        {
          if(i%2)
          {
            chassisLeft = rm_common::PointRotateAroundPoint(Point2D(RobotLength*0.5, RobotWidth*0.5) + chassisMid, chassisMid, chassisYaw - i * 3.14159 * 0.5);
            chassisRight = rm_common::PointRotateAroundPoint(Point2D(RobotLength*0.5, -RobotWidth*0.5) + chassisMid, chassisMid, chassisYaw - i * 3.14159 * 0.5);
          }
          else {

            chassisLeft = rm_common::PointRotateAroundPoint(Point2D(RobotWidth*0.5, RobotLength*0.5) + chassisMid, chassisMid, chassisYaw - i * 3.14159 * 0.5);
            chassisRight = rm_common::PointRotateAroundPoint(Point2D(RobotWidth*0.5, -RobotLength*0.5) + chassisMid, chassisMid, chassisYaw - i * 3.14159 * 0.5);
          }
            pointsfeedback->emplace_back(chassisLeft);
            pointsfeedback->emplace_back(chassisRight);
            chassisSidesMid_[i] = (chassisLeft + chassisRight) * 0.5;
            LineSegment2D chassisEdge(chassisLeft,chassisRight);
            if(rm_common::CheckLineSegmentsIntersection2D(chassisEdge,bulletSeg,&interSectionPoint))
            {
                isHit = true;
                  //if(rm_common::PointDistance(interSectionPoint,chassisSidesMid[i]) < AromrLength * 0.5)
                  //{
                        pointsIntersect.emplace_back(interSectionPoint);
                        sides.emplace_back(i);
                  //}
            }
        }
        if(!isHit)
            return 0;

        Point2D firstPointIntersect;
        double minDistance = 100;
        int index = 0;

        for(i=0;i<pointsIntersect.size();i++)
        {
            if(minDistance > rm_common::PointDistance(bullet.GetPositionLast(),pointsIntersect[i]))
            {
                minDistance = rm_common::PointDistance(bullet.GetPositionLast(),pointsIntersect[i]);
                firstPointIntersect = pointsIntersect[i];
                index = sides[i];
            }

        }
        if(rm_common::PointDistance(firstPointIntersect,chassisSidesMid_[index]) < AromrLength * 0.5)
            return index + 1;
        else {
            return index + 5;
        }
       // ROS_INFO("minDistanceIndex = %d,minDistanceIndex = %f",minDistanceIndex,minDistanceIndex);


    }




}

