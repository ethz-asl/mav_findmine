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

#include "fm_mission_planner/hotpoint_widget.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <memory>

#include <fm_trajectories/circle_trajectory.h>

namespace fm_mission_planner {

HotpointWidget::HotpointWidget(
    mav_trajectory_generation::InputConstraints input_constraints,
    QWidget* parent)
    : TrajectoryWidget(input_constraints, parent) {
  setWindowTitle("Hotpoint Trajectory");

  // Center parameters.
  east_edit_ = createDefaultSpinBox();
  north_edit_ = createDefaultSpinBox();
  select_center_button_ = new QPushButton(tr("Select"));
  select_center_button_->setToolTip(tr("Select the hotpoint center."));

  // Remaining parameters.
  altitude_edit_ = createDefaultSpinBox();
  altitude_edit_->setValue(4.0);

  radius_edit_ = createDefaultSpinBox();
  radius_edit_->setRange(0.1, 100.0);
  radius_edit_->setValue(3.0);

  start_heading_deg_edit_ = createDefaultAngleSpinBox();

  arc_length_deg_edit_ = createDefaultSpinBox();
  arc_length_deg_edit_->setValue(360.0);
  arc_length_deg_edit_->setRange(0.1, std::numeric_limits<double>::max());
  arc_length_deg_edit_->setSingleStep(5.0);

  circle_deviation_ratio_edit_ = createDefaultSpinBox();
  circle_deviation_ratio_edit_->setRange(0.001, 0.999);
  circle_deviation_ratio_edit_->setValue(0.01);
  circle_deviation_ratio_edit_->setSingleStep(0.01);
  circle_deviation_ratio_edit_->setDecimals(3);

  // The overall layout.
  QGridLayout* layout = new QGridLayout;

  layout->addWidget(new QLabel(tr("Circle Center [East, North]:")), 0, 0);
  layout->addWidget(east_edit_, 0, 1);
  layout->addWidget(north_edit_, 0, 2);
  layout->addWidget(select_center_button_, 0, 3);

  layout->addWidget(new QLabel(tr("Altitude Above Take Off [m]:")), 1, 0);
  layout->addWidget(altitude_edit_, 1, 1);

  layout->addWidget(new QLabel(tr("Radius [m]:")), 2, 0);
  layout->addWidget(radius_edit_, 2, 1);

  layout->addWidget(new QLabel(tr("Velocity [m/s]:")), 3, 0);
  layout->addWidget(velocity_edit_, 3, 1);

  layout->addWidget(new QLabel(tr("Start heading [deg]:")), 4, 0);
  layout->addWidget(start_heading_deg_edit_, 4, 1);
  layout->addWidget(new QLabel(tr("Offset heading [deg]:")), 4, 2);
  layout->addWidget(heading_offset_edit_, 4, 3);

  layout->addWidget(new QLabel(tr("Arc length [deg]:")), 5, 0);
  layout->addWidget(arc_length_deg_edit_, 5, 1);

  layout->addWidget(new QLabel(tr("Circle deviation ratio:")), 6, 0);
  layout->addWidget(circle_deviation_ratio_edit_, 6, 1);

  layout->addWidget(confirm_button_, 7, 3);

  setLayout(layout);

  // Connections.
  // Check if trajectory draft needs to be updated.
  connectAllSpinBoxes();

  // Center selection.
  connect(select_center_button_, &QPushButton::clicked, this,
          [this] { clickPoint(east_edit_, north_edit_); });
}

void HotpointWidget::updateTrajectory() {
  std::shared_ptr<fm_trajectories::CircleTrajectory::Settings> settings =
      std::make_shared<fm_trajectories::CircleTrajectory::Settings>(
          east_edit_->value(), north_edit_->value(), radius_edit_->value(),
          start_heading_deg_edit_->value(), arc_length_deg_edit_->value(),
          altitude_edit_->value(), velocity_edit_->value(),
          heading_offset_edit_->value(), prev_trajectory_, input_constraints_,
          circle_deviation_ratio_edit_->value());
  trajectory_ = std::make_shared<fm_trajectories::CircleTrajectory>(settings);
}

}  // namespace fm_mission_planner
