/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef HERO_COSTMAP_OBSTACLE_LAYER_H
#define HERO_COSTMAP_OBSTACLE_LAYER_H

#include <chrono>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/msg/point_cloud_conversion.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include "footprint.hpp"
#include "map_common.hpp"
#include "costmap_layer.hpp"
#include "layered_costmap.hpp"
#include "observation_buffer.hpp"
#include "map_common.hpp"

namespace hero_costmap {

class ObstacleLayer : public CostmapLayer
{
public:
  ObstacleLayer()
  {
    costmap_ = nullptr;
  }

  virtual ~ObstacleLayer() {}
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();
  virtual void updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                            double *max_x, double *max_y);
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message,
                         const std::shared_ptr<ObservationBuffer> &buffer);
  void laserScanValidInfoCallback(const sensor_msgs::msg::LaserScan::SharedPtr message,
                                  const std::shared_ptr<ObservationBuffer> &buffer);

protected:
  bool getMarkingObservations(std::vector<Observation> &marking_observations) const;
  bool getClearingObservations(std::vector<Observation> &clearing_observations) const;
  virtual void raytraceFreespace(const Observation &clearing_observation, double *min_x, double *min_y,
                                 double *max_x, double *max_y);
  void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double *min_x, double *min_y,
                            double *max_x, double *max_y);
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                       double *max_x, double *max_y);
  bool footprintClearingEnabled_, rollingWindow_;
  int combinationMethod_;
  std::string globalFrame_;
  double maxObstacleHeight_;
  std::vector<geometry_msgs::msg::Point> transformedFootprint_;
  laser_geometry::LaserProjection projector_;

  std::vector<std::shared_ptr<message_filters::SubscriberBase>> observationSubscribers_;
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observationNotifiers_;
  std::vector<std::shared_ptr<ObservationBuffer>> observationBuffers_;
  std::vector<std::shared_ptr<ObservationBuffer>> markingBuffers_;
  std::vector<std::shared_ptr<ObservationBuffer>> clearingBuffers_;

  std::vector<Observation> staticClearingObservations_, staticMarkingObservations_;
  std::chrono::system_clock::time_point resetTime_;
};;

} //namespace hero_costmap


#endif //HERO_COSTMAP_OBSTACLE_LAYER_H