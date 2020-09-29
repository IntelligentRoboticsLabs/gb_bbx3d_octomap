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

// Iterations Frequency:

#define FREQ 5

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
  auto bbx3d2octomaps_node = std::make_shared<bbx3d2octomaps::Bbx3D2Octomaps>(
    "bbx3d2octomaps_node");

  rclcpp::Rate loop_rate(FREQ);
  while (rclcpp::ok()) {
    bbx3d2octomaps_node->step();
    rclcpp::spin_some(bbx3d2octomaps_node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
