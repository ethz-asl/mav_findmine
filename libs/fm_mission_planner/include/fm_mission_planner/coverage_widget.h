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

#ifndef FM_MISSION_PLANNER_COVERAGE_WIDGET_H_
#define FM_MISSION_PLANNER_COVERAGE_WIDGET_H_

#ifndef Q_MOC_RUN
#include <polygon_coverage_msgs/PolygonWithHolesStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <vector>
#include "fm_mission_planner/trajectory_widget.h"
#endif

namespace fm_mission_planner {

class CoverageWidget : public TrajectoryWidget {
  Q_OBJECT

 public:
  CoverageWidget(mav_trajectory_generation::InputConstraints input_constraints,
                 QWidget* parent = 0);

 protected:
  void updateTrajectory() override;

 private:
  void selectPolygonTool();
  bool computeCoverageWaypoints(std::vector<Eigen::Vector3d>* pts);
  void polygonCallback(
      const polygon_coverage_msgs::PolygonWithHolesStamped& msg);

  QDoubleSpinBox* start_east_edit_;
  QDoubleSpinBox* start_north_edit_;
  QDoubleSpinBox* goal_east_edit_;
  QDoubleSpinBox* goal_north_edit_;
  QLabel* polygon_area_edit_;
  QLabel* polygon_altitude_edit_;
  QPushButton* select_start_button_;
  QPushButton* select_goal_button_;
  QPushButton* select_polygon_button_;
  QPushButton* generate_trajectory_btn_;

  QGridLayout* layout_;

  ros::Subscriber polygon_sub_;

  // States.
  bool expecting_polygon_ = false;
};

}  // end namespace fm_mission_planner

#endif  // FM_MISSION_PLANNER_COVERAGE_WIDGET_H_
