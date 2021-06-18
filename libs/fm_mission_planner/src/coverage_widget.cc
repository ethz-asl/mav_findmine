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

#include <fm_trajectories/waypoint_trajectory.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <polygon_coverage_geometry/cgal_comm.h>
#include <polygon_coverage_msgs/PlannerService.h>
#include <polygon_coverage_msgs/PolygonService.h>
#include <polygon_coverage_ros/ros_interface.h>
#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <memory>
#include "fm_mission_planner/coverage_widget.h"

namespace fm_mission_planner {

CoverageWidget::CoverageWidget(
    mav_trajectory_generation::InputConstraints input_constraints,
    QWidget* parent)
    : TrajectoryWidget(input_constraints, parent) {
  setWindowTitle("Coverage Trajectory");
  // Start point
  start_east_edit_ = createDefaultSpinBox();
  start_north_edit_ = createDefaultSpinBox();

  select_start_button_ = new QPushButton(tr("Select"));
  select_start_button_->setToolTip(tr("Select start vertex."));

  // Goal point
  goal_east_edit_ = createDefaultSpinBox();
  goal_north_edit_ = createDefaultSpinBox();

  select_goal_button_ = new QPushButton(tr("Select"));
  select_goal_button_->setToolTip(tr("Select goal vertex."));

  // Polygon
  select_polygon_button_ = new QPushButton(tr("Select"));
  select_polygon_button_->setToolTip(tr("Select polygon."));
  polygon_area_edit_ = new QLabel(QString::number(0.0));
  polygon_altitude_edit_ = new QLabel(QString::number(0.0));

  // Disable confirm button.
  confirm_button_->setEnabled(false);

  // The overall layout.
  layout_ = new QGridLayout;

  layout_->addWidget(new QLabel(tr("Start [East m]:")), 0, 0);
  layout_->addWidget(start_east_edit_, 0, 1);
  layout_->addWidget(new QLabel(tr("Start [North m]:")), 0, 2);
  layout_->addWidget(start_north_edit_, 0, 3);
  layout_->addWidget(select_start_button_, 0, 4);

  layout_->addWidget(new QLabel(tr("Goal [East m]:")), 1, 0);
  layout_->addWidget(goal_east_edit_, 1, 1);
  layout_->addWidget(new QLabel(tr("Goal [North m]:")), 1, 2);
  layout_->addWidget(goal_north_edit_, 1, 3);
  layout_->addWidget(select_goal_button_, 1, 4);

  layout_->addWidget(new QLabel(tr("Polygon Area [m^2]:")), 2, 0);
  layout_->addWidget(polygon_area_edit_, 2, 1);
  layout_->addWidget(new QLabel(tr("Altitude [m]:")), 2, 2);
  layout_->addWidget(polygon_altitude_edit_, 2, 3);
  layout_->addWidget(select_polygon_button_, 2, 4);

  layout_->addWidget(new QLabel(tr("Velocity [m/s]:")), 4, 0);
  layout_->addWidget(velocity_edit_, 4, 1);

  layout_->addWidget(new QLabel(tr("Offset heading [deg]:")), 4, 2);
  layout_->addWidget(heading_offset_edit_, 4, 3);

  layout_->addWidget(new QLabel(tr("Trajectory time [s]:")), 5, 0);
  layout_->addWidget(trajectory_time_edit_, 5, 1);

  generate_trajectory_btn_ = new QPushButton(tr("Generate trajectory"));
  layout_->addWidget(generate_trajectory_btn_, 5, 4);

  layout_->addWidget(confirm_button_, 6, 4);

  setLayout(layout_);

  // Connections.
  connect(select_start_button_, &QPushButton::clicked, this,
          [this] { clickPoint(start_east_edit_, start_north_edit_); });
  connect(select_goal_button_, &QPushButton::clicked, this,
          [this] { clickPoint(goal_east_edit_, goal_north_edit_); });
  connect(select_polygon_button_, &QPushButton::clicked, this,
          [this] { selectPolygonTool(); });
  connect(generate_trajectory_btn_, &QPushButton::clicked, this,
          [this] { updateTrajectory(); });

  polygon_sub_ =
      nh_.subscribe("polygon", 1, &CoverageWidget::polygonCallback, this);
}

void CoverageWidget::selectPolygonTool() {
  rviz::ToolManager* tool_man = vis_manager_->getToolManager();
  tool_man->setCurrentTool(tool_man->getTool(8));
  expecting_polygon_ = true;
}

void CoverageWidget::polygonCallback(
    const polygon_coverage_msgs::PolygonWithHolesStamped& msg) {
  if (expecting_polygon_) {
    polygon_coverage_msgs::PolygonService srv;
    srv.request.polygon = msg;
    auto coverage_planner_client =
        nh_.serviceClient<polygon_coverage_msgs::PolygonService>(
            "coverage_planner/set_polygon");
    if (!coverage_planner_client.call(srv) || !srv.response.success) {
      ROS_ERROR("Failed planning coverage trajectory.");
      ROS_ERROR("Please select another polygon.");
      return;
    }

    PolygonWithHoles pwh;
    double altitude;
    std::string frame;
    if (polygon_coverage_planning::polygonFromMsg(msg, &pwh, &altitude,
                                                  &frame)) {
      polygon_area_edit_->setText(QString::number(
          CGAL::to_double(polygon_coverage_planning::computeArea(pwh))));
      polygon_altitude_edit_->setText(QString::number(altitude));
    }
    expecting_polygon_ = false;
  }
}

void CoverageWidget::updateTrajectory() {
  // Delete old markers.
  if (trajectory_ != nullptr)
    trajectory_draft_pub_.publish(trajectory_->getTrajectoryDeleteMarkers());

  // Create polygon.
  std::vector<Eigen::Vector3d> pts;
  if (!computeCoverageWaypoints(&pts)) return;
  ROS_INFO_STREAM("Number of waypoints: " << pts.size());

  // Create polygonomial trajectory.
  std::shared_ptr<fm_trajectories::WaypointTrajectory::Settings> settings =
      std::make_shared<fm_trajectories::WaypointTrajectory::Settings>(
          pts, velocity_edit_->value(), heading_offset_edit_->value(),
          prev_trajectory_, input_constraints_);
  trajectory_ = std::make_shared<fm_trajectories::WaypointTrajectory>(settings);

  if (trajectory_ != nullptr) {
    trajectory_draft_pub_.publish(trajectory_->getTrajectoryMarkers());
    // Update trajectory time.
    trajectory_time_edit_->setText(QString::number(trajectory_->getTime()));
    confirm_button_->setEnabled(true);
  }
}

bool CoverageWidget::computeCoverageWaypoints(
    std::vector<Eigen::Vector3d>* pts) {
  polygon_coverage_msgs::PlannerService planner_srv;
  planner_srv.request.start_pose.pose.position.x = start_east_edit_->value();
  planner_srv.request.start_pose.pose.position.y = start_north_edit_->value();

  planner_srv.request.goal_pose.pose.position.x = goal_east_edit_->value();
  planner_srv.request.goal_pose.pose.position.y = goal_north_edit_->value();

  auto coverage_planner_client =
      nh_.serviceClient<polygon_coverage_msgs::PlannerService>(
          "coverage_planner/plan_path");
  if (!coverage_planner_client.call(planner_srv) |
      !planner_srv.response.success) {
    ROS_ERROR("Failed planning coverage trajectory.");
    return false;
  }

  mav_msgs::EigenTrajectoryPointVector traj;
  mav_msgs::eigenTrajectoryPointVectorFromMsg(planner_srv.response.sampled_plan,
                                              &traj);

  CHECK_NOTNULL(pts);
  pts->clear();
  for (const mav_msgs::EigenTrajectoryPoint& p : traj) {
    pts->push_back(p.position_W);
  }

  return true;
}

}  // namespace fm_mission_planner
