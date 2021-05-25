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
#include "static_layer_setting.pb.h"
#include "static_layer.h"
#include <thread>

namespace roborts_costmap {

void StaticLayer::OnInitialize() {
  ros::NodeHandle nh;
  is_current_ = true;
  ParaStaticLayer para_static_layer;

  std::string static_map = ros::package::getPath("roborts_costmap") + \
      "/config/static_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(static_map.c_str(), &para_static_layer);
  global_frame_ = layered_costmap_-> GetGlobalFrameID();
  first_map_only_ = para_static_layer.first_map_only();
  subscribe_to_updates_ = para_static_layer.subscribe_to_updates();
  track_unknown_space_ = para_static_layer.track_unknown_space();
  use_maximum_ = para_static_layer.use_maximum();
  int temp_threshold = para_static_layer.lethal_threshold();
  lethal_threshold_ = std::max(std::min(100, temp_threshold), 0);
  trinary_costmap_ = para_static_layer.trinary_map();
  unknown_cost_value_ = para_static_layer.unknown_cost_value();
  map_received_ = false;
  bool is_debug_ = para_static_layer.is_debug();
  map_topic_ = para_static_layer.topic_name();
  map_sub_ = nh.subscribe(map_topic_.c_str(), 1, &StaticLayer::InComingMap, this);
  ref_sub_ = nh.subscribe("game_zone_array_status", 1, &StaticLayer::RefereeCB, this);
  ros::Rate temp_rate(10);
  //ros::AsyncSpinner spinner(2);
  while(!map_received_ && !map_reset_) {
    ros::spinOnce();
  }
  temp_rate.sleep();
  map_update_ = false;
  while(!map_update_ && map_reset_) {
    ros::spinOnce();
  }

  if (map_reset_) {
    ROS_INFO("add prohibition area!");

    unsigned int temp_index = 0;
    unsigned char value = 100;
    float resolution_x = 8.72;
    float resulution_y = 8.91;

    for (auto i = 0; i < height_; i++) {
      for (auto j = 0; j < width_; j++) {
        if (j < int((530 + 540) / resolution_x) && j > int(530 / resolution_x) && i > int(2850 / resulution_y) && i < int((2850 + 480) / resulution_y)) {
          if (prohibition_[0] == 1) 
            costmap_[temp_index] = InterpretValue(value);
          else
            costmap_[temp_index] = InterpretValue(0);
        }

        if (j < int((1930 + 540) / resolution_x) && j > int(1930 / resolution_x) && i > int(1710 / resulution_y) && i < int((1710 + 480) / resulution_y)) {
          if (prohibition_[1] == 1) 
            costmap_[temp_index] = InterpretValue(value);
          else
            costmap_[temp_index] = InterpretValue(0);
        }

        if (j < int((4070 + 540) / resolution_x) && j > int(4070 / resolution_x) && i > int(4095 / resulution_y) && i < int((4095 + 480) / resulution_y)) {
          if (prohibition_[2] == 1) 
            costmap_[temp_index] = InterpretValue(value);
          else
            costmap_[temp_index] = InterpretValue(0);
        }

        if (j < int((4070 + 540) / resolution_x) && j > int(4070 / resolution_x) && i > int(505 / resulution_y) && i < int((505 + 480) / resulution_y)) {
          if (prohibition_[3] == 1) 
            costmap_[temp_index] = InterpretValue(value);
          else
            costmap_[temp_index] = InterpretValue(0);
        }

        if (j < int((6210 + 540) / resolution_x) && j > int(6210 / resolution_x) && i > int(2890 / resulution_y) && i < int((2890 + 480) / resulution_y)) {
          if (prohibition_[4] == 1) 
            costmap_[temp_index] = InterpretValue(value);
          else
            costmap_[temp_index] = InterpretValue(0);
        }

        if (j < int((7610 + 540) / resolution_x) && j > int(7610 / resolution_x) && i > int(1750 / resulution_y) && i < int((1750 + 480) / resulution_y)) {
          if (prohibition_[5] == 1) 
            costmap_[temp_index] = InterpretValue(value);
          else
            costmap_[temp_index] = InterpretValue(0);
        }

        ++temp_index;
      }
    }
  }
  
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  is_enabled_ = true;
  has_updated_data_ = true;
}

void StaticLayer::MatchSize() {
  if (!layered_costmap_->IsRolling()) {
    Costmap2D* master = layered_costmap_->GetCostMap();
    ResizeMap(master->GetSizeXCell(), master->GetSizeYCell(), master->GetResolution(),
              master->GetOriginX(), master->GetOriginY());
  }
}

void StaticLayer::InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map) {
  unsigned int temp_index = 0;
  unsigned char value = 0;

  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
  auto resolution = new_map->info.resolution;
  auto origin_x = new_map->info.origin.position.x;
  auto origin_y = new_map->info.origin.position.y;
  auto master_map = layered_costmap_->GetCostMap();
  if(!layered_costmap_->IsRolling() && (master_map->GetSizeXCell() != size_x || master_map->GetSizeYCell() != size_y ||
      master_map->GetResolution() != resolution || master_map->GetOriginX() != origin_x || master_map->GetOriginY() != origin_y ||
      !layered_costmap_->IsSizeLocked())) {
    layered_costmap_->ResizeMap(size_x, size_y, resolution, origin_x, origin_y, true);
  } else if(size_x_ != size_x || size_y_ != size_y || resolution_ != resolution || origin_x_ != origin_x || origin_y_ != origin_y) {
    ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }
  //std::cout << size_x << std::endl;
  //std::cout << size_y << std::endl;
  for (auto i = 0; i < size_y; i++) {
    for (auto j = 0; j < size_x; j++) {
      value = new_map->data[temp_index];
      costmap_[temp_index] = InterpretValue(value);
      ++temp_index;
    }
  }
  map_received_ = true;
  has_updated_data_ = true;
  map_frame_ = new_map->header.frame_id;
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  if (first_map_only_) {
    map_sub_.shutdown();
  }
}

void IncomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update) {
  ROS_INFO("123");
}

unsigned char StaticLayer::InterpretValue(unsigned char value) {
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::Activate() {
  OnInitialize();
}

void StaticLayer::Deactivate() {
//    delete cost_map_;
  //shut down the map topic message subscriber
  map_sub_.shutdown();
}

void StaticLayer::Reset() {
  if(first_map_only_) {
    has_updated_data_ = true;
  } else {
    map_reset_ = true;
    map_update_ = false;
    
    OnInitialize();
  }
}

void StaticLayer::UpdateBounds(double robot_x,
                               double robot_y,
                               double robot_yaw,
                               double *min_x,
                               double *min_y,
                               double *max_x,
                               double *max_y) {
  double wx, wy;
  if(!layered_costmap_->IsRollingWindow()) {
    if(!map_received_ || !(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }
  //just make sure the value is normal
  UseExtraBounds(min_x, min_y, max_x, max_y);
  Map2World(staic_layer_x_, staic_layer_y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  Map2World(staic_layer_x_+ width_, staic_layer_y_ + height_, wx, wy);
  *max_x = std::max(*max_x, wx);
  *max_y = std::max(*max_y, wy);
  has_updated_data_ = false;
}

void StaticLayer::UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
  if(!map_received_) {
    return;
  }
  if(!layered_costmap_->IsRollingWindow()) {
    if(!use_maximum_) {
      UpdateOverwriteByAll(master_grid, min_i, min_j, max_i, max_j);
    } else {
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else {
    unsigned int mx, my;
    double wx, wy;
    tf::StampedTransform temp_transform;
    try {
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), temp_transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    for(auto i = min_i; i < max_i; ++i) {
      for(auto j = min_j; j < max_j; ++j) {
        layered_costmap_->GetCostMap()->Map2World(i, j, wx, wy);
        tf::Point p(wx, wy, 0);
        p = temp_transform(p);
        if(World2Map(p.x(), p.y(), mx, my)){
          if(!use_maximum_) {
            master_grid.SetCost(i, j, GetCost(mx, my));
          }
          else {
            master_grid.SetCost(i, j, std::max(master_grid.GetCost(i, j), GetCost(i, j)));
          }
        }
      }
    }
  }
}

void StaticLayer::RefereeCB(const roborts_msgs::GameZoneArray::ConstPtr &zone) {
  zone_ = *zone;
  unsigned char prohibition_red_[4] = {2, 1, 5, 4};
  //std::cout << zone_.zone[0].type << std::endl;
  //std::cout << prohibition_red_[0] << std::endl;

  for (int i = 0; i < 6; i++) {
    if (zone_.zone[i].type == prohibition_red_[0] || zone_.zone[i].type == prohibition_red_[1] 
     || zone_.zone[i].type == prohibition_red_[2] || zone_.zone[i].type == prohibition_red_[3]) {
      prohibition_[i] = 1;
    }
    else 
      prohibition_[i] = 0;
  }
  map_update_ = true;
}

} //namespace roborts_costmap

