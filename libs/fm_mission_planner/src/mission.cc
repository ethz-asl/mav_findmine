/*
MIT License

Copyright (c) 2020 Rik Baehnemann, ASL, ETH Zurich, Switzerland

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "fm_mission_planner/mission.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <memory>
#include <string>

#include <mav_trajectory_generation/io.h>
#include <visualization_msgs/MarkerArray.h>

namespace mtg = mav_trajectory_generation;

namespace fm_mission_planner {

const bool kConstantVelocity = false;

void Mission::addTrajectory(
    const std::shared_ptr<fm_trajectories::BaseTrajectory>& trajectory) {
  trajectory_vec_.push_back(trajectory);
}

void Mission::setInputConstraints(
    const mav_trajectory_generation::InputConstraints& input_constraints) {
  input_constraints_ = input_constraints;
}

void Mission::clear() { trajectory_vec_.clear(); }

mav_trajectory_generation::InputConstraints Mission::getInputConstraints()
    const {
  return input_constraints_;
}

void Mission::setGeodeticReference(double lat, double lon, double alt) {
  reference_set_ = true;
  lat_ = lat;
  lon_ = lon;
  alt_ = alt;
}

visualization_msgs::MarkerArray Mission::getTrajectoryMarkers() const {
  visualization_msgs::MarkerArray markers;

  size_t i = 0;
  for (const std::shared_ptr<fm_trajectories::BaseTrajectory> t :
       trajectory_vec_) {
    visualization_msgs::MarkerArray t_markers =
        t->getTrajectoryMarkers("trajectory_" + std::to_string(i++));
    markers.markers.insert(markers.markers.end(), t_markers.markers.begin(),
                           t_markers.markers.end());
  }

  return markers;
}

visualization_msgs::MarkerArray Mission::getTrajectoryDeleteMarkers() const {
  visualization_msgs::MarkerArray markers;

  for (const std::shared_ptr<fm_trajectories::BaseTrajectory> t :
       trajectory_vec_) {
    visualization_msgs::MarkerArray t_markers = t->getTrajectoryDeleteMarkers();
    markers.markers.insert(markers.markers.end(), t_markers.markers.begin(),
                           t_markers.markers.end());
  }

  return markers;
}

std::string Mission::toString() const {
  if (!reference_set_) {
    ROS_WARN("Geodetic reference not set.");
    return "";
  }

  YAML::Node node;
  // Trajectories.
  for (size_t i = 0; i < trajectory_vec_.size(); ++i) {
    YAML::Node trajectory_node;
    trajectory_vec_[i]->setGeodeticReference(lat_, lon_, alt_);
    if (!trajectory_vec_[i]->toYaml(&trajectory_node)) {
      ROS_ERROR("Cannot convert trajectory to YAML.");
      return "";
    }
    node["trajectory_" + std::to_string(i)] = trajectory_node;
  }
  // Input constraints.
  node["input_constraints"] = input_constraints_.toYaml();

  // // TODO(rikba): Sort alphabetically.
  // if (node.IsMap()) {
  //   std::sort(
  //       node.begin(), node.end(), [=](YAML::iterator a, YAML::iterator b) {
  //         return ((a->first.as<std::string>()) <
  //         (b->first.as<std::string>()));
  //       });
  // }

  std::ostringstream out;
  out << node;
  return out.str();
}

std::string Mission::exportTrajectory() const {
  // Concatenate polynomial trajectory.
  mtg::Trajectory trajectory;
  for (size_t i = 0; i < trajectory_vec_.size(); ++i) {
    if (!trajectory_vec_[i]) {
      ROS_WARN_STREAM("Trajectory " << i << " not set.");
      return "";
    }

    mtg::Trajectory partial_trajectory;
    if (!trajectory_vec_[i]->planTrajectory(&partial_trajectory)) {
      ROS_WARN_STREAM("Cannot receive trajectory " << i << ".");
      return "";
    }
    mtg::Trajectory temp_trajectory = trajectory;
    if (i == 0) {
      trajectory = partial_trajectory;
    } else if (!temp_trajectory.addTrajectories({partial_trajectory},
                                                &trajectory)) {
      ROS_WARN_STREAM("Failed to add partial trajectory " << i << ".");
      return "";
    }
  }
  // Polynomial trajectory to YAML.
  YAML::Node node;
  node["trajectory"] = mtg::trajectoryToYaml(trajectory);
  node["input_constraints"] = input_constraints_.toYaml();
  YAML::Node geodetic_reference_node;
  if (!referenceToYaml(&geodetic_reference_node)) return "";
  node["geodetic_reference"] = geodetic_reference_node;

  std::ostringstream out;
  out << node;
  return out.str();
}

bool Mission::referenceToYaml(YAML::Node* node) const {
  CHECK_NOTNULL(node);
  *node = YAML::Node();

  if (!reference_set_) {
    ROS_WARN("Geodetic reference not set.");
    return false;
  }

  (*node)["lat_enu"] = lat_;
  (*node)["lon_enu"] = lon_;
  (*node)["alt_enu"] = alt_;

  return true;
}

}  // namespace fm_mission_planner
