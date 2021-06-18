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

#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <limits>
#include "fm_mission_planner/trajectory_widget.h"

namespace fm_mission_planner {

TrajectoryWidget::TrajectoryWidget(
    mav_trajectory_generation::InputConstraints input_constraints,
    QWidget* parent)
    : QDialog(parent),
      trajectory_(nullptr),
      input_constraints_(input_constraints),
      nh_private_("~"),
      expecting_clicked_point_(false),
      expecting_2d_nav_goal_(false) {
  clicked_point_sub_ = nh_.subscribe(
      "/clicked_point", 1, &TrajectoryWidget::clickedPointCallback, this);
  nav_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
                                &TrajectoryWidget::navGoalCallback, this);
  trajectory_draft_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("trajectory_draft",
                                                             1);
  // Common parameters.
  velocity_edit_ = createDefaultSpinBox();
  double v_max = std::numeric_limits<double>::max();
  input_constraints_.getConstraint(
      mav_trajectory_generation::InputConstraintType::kVMax, &v_max);
  velocity_edit_->setRange(-v_max, v_max);
  velocity_edit_->setValue(1.5);
  velocity_edit_->setSingleStep(0.1);

  heading_offset_edit_ = createDefaultAngleSpinBox();

  trajectory_time_edit_ = new QLabel(QString::number(0.0));

  // Confirmation button.
  confirm_button_ = new QPushButton(tr("Confirm"));
  confirm_button_->setToolTip(
      tr("Confirm the trajectory when all parameters are set."));
  connect(confirm_button_, &QPushButton::clicked, this,
          [this] { sendTrajectory(); });
}

void TrajectoryWidget::checkSpinBoxes() {
  // Delete old markers.
  if (trajectory_ != nullptr)
    trajectory_draft_pub_.publish(trajectory_->getTrajectoryDeleteMarkers());
  // Publish new trajectory.
  updateTrajectory();
  if (trajectory_ != nullptr) {
    trajectory_draft_pub_.publish(trajectory_->getTrajectoryMarkers());
    // Update trajectory time.
    trajectory_time_edit_->setText(QString::number(trajectory_->getTime()));
  }
}

void TrajectoryWidget::connectAllSpinBoxes() {
  // TODO(rikba): Don't update trajectory twice when selecting point.
  QList<QDoubleSpinBox*> spin_box_list = findChildren<QDoubleSpinBox*>();
  for (const QDoubleSpinBox* sb : spin_box_list)
    connect(sb,
            (void (QDoubleSpinBox::*)(double)) & QDoubleSpinBox::valueChanged,
            this, [this] { checkSpinBoxes(); });
}

void TrajectoryWidget::sendTrajectory() {
  // make sure the trajectory is not null when called
  if (trajectory_ != nullptr) {
    trajectory_draft_pub_.publish(trajectory_->getTrajectoryDeleteMarkers());
    Q_EMIT outputTrajectory(trajectory_);
    Q_EMIT accepted();
    trajectory_.reset();
    close();
  } else {
    ROS_WARN_STREAM("trajectory is null ptr ");
  }
}

void TrajectoryWidget::clickPoint(QDoubleSpinBox* click_x,
                                  QDoubleSpinBox* click_y) {
  click_x_ = click_x;
  click_y_ = click_y;
  rviz::ToolManager* tool_man = vis_manager_->getToolManager();
  tool_man->setCurrentTool(tool_man->getTool(7));
  expecting_clicked_point_ = true;
}

void TrajectoryWidget::clickNavGoal(QDoubleSpinBox* click_x,
                                    QDoubleSpinBox* click_y,
                                    QDoubleSpinBox* click_delta) {
  click_x_ = click_x;
  click_y_ = click_y;
  click_delta_ = click_delta;
  rviz::ToolManager* tool_man = vis_manager_->getToolManager();
  tool_man->setCurrentTool(tool_man->getTool(6));
  expecting_2d_nav_goal_ = true;
}

void TrajectoryWidget::clickedPointCallback(
    const geometry_msgs::PointStampedConstPtr& msg) {
  if (expecting_clicked_point_) {
    CHECK_NOTNULL(click_x_);
    CHECK_NOTNULL(click_y_);
    click_x_->setValue(msg->point.x);
    click_y_->setValue(msg->point.y);
    expecting_clicked_point_ = false;
  }
}

void TrajectoryWidget::navGoalCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  if (expecting_2d_nav_goal_) {
    CHECK_NOTNULL(click_x_);
    CHECK_NOTNULL(click_y_);
    CHECK_NOTNULL(click_delta_);
    click_x_->setValue(msg->pose.position.x);
    click_y_->setValue(msg->pose.position.y);
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(msg->pose.orientation, q);
    Eigen::AngleAxisd aa(q);
    double delta =
        aa.axis() == Eigen::Vector3d::UnitZ() ? aa.angle() : -aa.angle();
    delta = angles::normalize_angle_positive(delta);
    click_delta_->setValue(angles::to_degrees(delta));
    expecting_2d_nav_goal_ = false;
  }
}

void TrajectoryWidget::setVisualizationManager(
    rviz::VisualizationManager* vis_manager) {
  vis_manager_ = vis_manager;
}

void TrajectoryWidget::setPreviousTrajectory(
    const std::weak_ptr<fm_trajectories::BaseTrajectory>& prev_trajectory) {
  prev_trajectory_ = prev_trajectory;
}

QDoubleSpinBox* TrajectoryWidget::createDefaultSpinBox() {
  QDoubleSpinBox* box = new QDoubleSpinBox();
  box->setDecimals(2);
  box->setSingleStep(0.1);
  box->setRange(std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::max());
  return box;
}

QDoubleSpinBox* TrajectoryWidget::createDefaultAngleSpinBox() {
  QDoubleSpinBox* box = createDefaultSpinBox();
  box->setSingleStep(5.0);
  box->setWrapping(true);
  box->setRange(0.0, 360.0);
  return box;
}

void TrajectoryWidget::reject() {
  if (trajectory_ != nullptr)
    trajectory_draft_pub_.publish(trajectory_->getTrajectoryDeleteMarkers());
  trajectory_.reset();
  QDialog::reject();
}

}  // namespace fm_mission_planner
