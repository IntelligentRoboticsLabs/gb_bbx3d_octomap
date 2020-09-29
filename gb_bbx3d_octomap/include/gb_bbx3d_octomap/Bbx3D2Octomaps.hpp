// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Francisco Martín fmrico@gmail.com */
/* Author: Fernando González fergonzaramos@yahoo.es */
/* Author: José Miguel Guerrero josemiguel.guerrero@urjc.es */

#ifndef BBX3D2OCTOMAPS__BBX3D2OCTOMAPS_HPP_
#define BBX3D2OCTOMAPS__BBX3D2OCTOMAPS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/conversions.h>
#include <math.h>
#include <vector>
#include <string>
#include <octomap_msgs/msg/octomap.hpp>
#include "gb_visual_detection_3d_msgs/msg/bounding_boxes3d.hpp"
#include "gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp"

namespace bbx3d2octomaps
{

class Bbx3D2Octomaps : public rclcpp::Node
{
public:
  Bbx3D2Octomaps(std::string name);
  void step();

private:
  void setOctomap();
  void updateOctomap();
  void publishFullOctoMap();
  void bboxes3dCb(const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d::SharedPtr msg);
  void buildOctomap(const gb_visual_detection_3d_msgs::msg::BoundingBox3d & bbox,
    octomap::point3d * point);

  bool isNewCell(double x, double y, double z, double margin_error);

  std::shared_ptr<octomap::OcTree> octree_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
  rclcpp::Subscription
    <gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>::SharedPtr yolact3d_sub_;

  std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> bboxes_;
  std::string frame_id_;
  octomap::KeySet cells_;
};

}  // namespace bbx3d2octomaps

#endif  // BBX3D2OCTOMAPS__BBX3D2OCTOMAPS_HPP_
