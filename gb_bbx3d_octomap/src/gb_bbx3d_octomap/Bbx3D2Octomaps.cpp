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

#include "gb_bbx3d_octomap/Bbx3D2Octomaps.hpp"
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace bbx3d2octomaps {

Bbx3D2Octomaps::Bbx3D2Octomaps(std::string name)
:
Node(name)
{
	double probHit, probMiss, thresMin, thresMax;
	double voxel_res = 0.1;
  probHit = 0.7;
  probMiss = 0.4;
  thresMin = 0.12;
  thresMax = 0.97;

	octree_ = std::make_shared<octomap::OcTree>(voxel_res);
  octree_->setProbHit(probHit);
  octree_->setProbMiss(probMiss);
  octree_->setClampingThresMin(thresMin);
  octree_->setClampingThresMax(thresMax);

  yolact3d_sub_ = this->create_subscription<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
    "/yolact_ros2_3d/bounding_boxes_3d", 1,
    std::bind(&Bbx3D2Octomaps::bboxes3dCb, this, std::placeholders::_1));

  pub_ = create_publisher<octomap_msgs::msg::Octomap>("/dummy_octomap", rclcpp::SystemDefaultsQoS());
}

bool
Bbx3D2Octomaps::isNewCell(double x, double y, double z, double margin_error)
{
  /*
   * Returns if cell centered on (x,y,z) is new or just exists and if exists,
   * its occupancy probability is incremented
   */

  bool found = false;
  float prob;

  for (auto it = octree_->begin_leafs(), end=octree_->end_leafs();
    it!= end && !found; ++it)
  {
    if (fabs(it.getX() - x) <= margin_error && fabs(it.getY() - y) <= margin_error &&
      fabs(it.getZ() - z) <= margin_error)
    {
      found = true;

      prob = it->getValue() + 0.2f;
      prob = std::min(1.0f, prob);
      octree_->updateInnerOccupancy();
      octree_->updateNode(it.getKey(), prob);
    }
  }

  return !found;
}

void
Bbx3D2Octomaps::bboxes3dCb(
  const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d::SharedPtr msg)
{
  bboxes_ = msg->bounding_boxes;
  frame_id_ = msg->header.frame_id;
}

void
Bbx3D2Octomaps::buildOctomap(const gb_visual_detection_3d_msgs::msg::BoundingBox3d & bbox,
  octomap::point3d * point)
{
  double step = 0.1;

  for (double y = bbox.ymin; y < bbox.ymax; y+= step) {
    for (double z = bbox.zmin; z < bbox.zmax; z+= step) {
      for (double x = bbox.xmin; x < bbox.xmax; x+= step) {
        if (!isNewCell(x, y, z, step / 2.0))
          continue;
        *point = octomap::point3d(x, y, z);
        octree_->updateInnerOccupancy();
        octree_->setNodeValue(*point, 0.6f, false);
      }
    }
  }
}

void
Bbx3D2Octomaps::setOctomap()
{
  octomap::point3d point;

  /*
   * Habria primero que cambiar de frame el bonding box
   */


  for(auto bbox : bboxes_) {
    if(bbox.object_name != "person")
      continue;
    buildOctomap(bbox, &point);
  }
}

void
Bbx3D2Octomaps::publishFullOctoMap()
{
	octomap_msgs::msg::Octomap map;
  size_t octomapSize;

  map.header.frame_id = frame_id_;
  octomapSize = octree_->size();
  if (octomapSize <= 1) {
    RCLCPP_WARN(get_logger(),"Nothing to publish, octree is empty");
    //return;
  }
  if (octomap_msgs::fullMapToMsg(*octree_, map)) {
    pub_->publish(map);
    RCLCPP_INFO(get_logger(), "publishing an octomap of size [%u]", octomapSize);
  } else {
    RCLCPP_ERROR(get_logger(),"Error serializing OctoMap");
  }
}

void
Bbx3D2Octomaps::updateOctomap()
{
  float prob;

  for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end=octree_->end_leafs();
    it != end; ++it)
  {
    prob = it->getValue();
    if (prob <= 0.12) {
      octree_->deleteNode(it.getKey(), it.getDepth());
    } else {
      prob -= 0.3;
      octree_->updateInnerOccupancy();
      octree_->setNodeValue(it.getKey(), prob, false);
    }
  }
}

void
Bbx3D2Octomaps::step()
{
  updateOctomap();
  setOctomap();
  publishFullOctoMap();
}

}  // namespace bbx3d2octomaps
