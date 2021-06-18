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

#include "fm_mission_planner/strip_widget.h"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <memory>

#include <fm_trajectories/strip_trajectory.h>

namespace fm_mission_planner {

StripWidget::StripWidget(
    mav_trajectory_generation::InputConstraints input_constraints,
    QWidget* parent)
    : TrajectoryWidget(input_constraints, parent) {
  setWindowTitle("Strip Trajectory");

  // Vertex A parameters.
  A_east_edit_ = createDefaultSpinBox();
  A_north_edit_ = createDefaultSpinBox();
  delta_deg_edit_ = createDefaultAngleSpinBox();

  select_A_delta_button_ = new QPushButton(tr("Select"));
  select_A_delta_button_->setToolTip(
      tr("Select vertex A and direction of edge a."));

  // Remaining parameters.
  offset_edit_ = createDefaultSpinBox();

  a_edit_ = createDefaultSpinBox();
  a_edit_->setMinimum(0.1);
  a_edit_->setValue(1.0);

  for (size_t i = 0; i < 2; ++i) {
    altitudes_.emplace_back(createDefaultSpinBox());
    altitudes_.back()->setValue(0.0);
  }
  altitudes_[0]->setValue(4.0);

  // TODO(rikba): Make integer spin box.
  num_revolutions_edit_ = createDefaultSpinBox();
  num_revolutions_edit_->setDecimals(0);
  num_revolutions_edit_->setMinimum(1);
  num_revolutions_edit_->setValue(1);
  num_revolutions_edit_->setSingleStep(1);

  start_vertex_edit_ = createDefaultSpinBox();
  start_vertex_edit_->setDecimals(0);
  start_vertex_edit_->setRange(0, 1);
  start_vertex_edit_->setSingleStep(1);
  start_vertex_edit_->setWrapping(true);

  velocity_edit_->setMinimum(0.0);

  // The overall layout.
  QGridLayout* layout = new QGridLayout;

  layout->addWidget(new QLabel(tr("Vertex A [East m]:")), 0, 0);
  layout->addWidget(A_east_edit_, 0, 1);
  layout->addWidget(new QLabel(tr("Vertex A [North m]:")), 0, 2);
  layout->addWidget(A_north_edit_, 0, 3);
  layout->addWidget(new QLabel(tr("Direction a [ENU degree]:")), 0, 4);
  layout->addWidget(delta_deg_edit_, 0, 5);
  layout->addWidget(select_A_delta_button_, 0, 6);

  layout->addWidget(new QLabel(tr("Perpendicular Offset [m]:")), 1, 0);
  layout->addWidget(offset_edit_, 1, 1);

  layout->addWidget(new QLabel(tr("Edge length a [m]:")), 2, 0);
  layout->addWidget(a_edit_, 2, 1);

  layout->addWidget(new QLabel(tr("Absolute height A [m]:")), 3, 0);
  layout->addWidget(altitudes_[0], 3, 1);

  layout->addWidget(new QLabel(tr("Relative height B [m]:")), 4, 0);
  layout->addWidget(altitudes_[1], 4, 1);

  layout->addWidget(new QLabel(tr("Velocity [m/s]:")), 5, 0);
  layout->addWidget(velocity_edit_, 5, 1);

  layout->addWidget(new QLabel(tr("Offset heading [deg]:")), 6, 0);
  layout->addWidget(heading_offset_edit_, 6, 1);

  layout->addWidget(new QLabel(tr("Start vertex:")), 7, 0);
  layout->addWidget(start_vertex_edit_, 7, 1);

  layout->addWidget(new QLabel(tr("Number of repetitions:")), 8, 0);
  layout->addWidget(num_revolutions_edit_, 8, 1);

  layout->addWidget(confirm_button_, 9, 6);

  setLayout(layout);

  // Connections.
  // Check if trajectory draft needs to be updated.
  connectAllSpinBoxes();

  // Vertex direction selection.
  connect(select_A_delta_button_, &QPushButton::clicked, this, [this] {
    clickNavGoal(A_east_edit_, A_north_edit_, delta_deg_edit_);
  });
}

void StripWidget::updateTrajectory() {
  std::shared_ptr<fm_trajectories::StripTrajectory::Settings> settings =
      std::make_shared<fm_trajectories::StripTrajectory::Settings>(
          A_east_edit_->value(), A_north_edit_->value(),
          delta_deg_edit_->value(), a_edit_->value(), altitudes_[0]->value(),
          altitudes_[1]->value(), offset_edit_->value(),
          velocity_edit_->value(), heading_offset_edit_->value(),
          prev_trajectory_, input_constraints_,
          static_cast<int>(start_vertex_edit_->value()),
          static_cast<int>(num_revolutions_edit_->value()));
  trajectory_ = std::make_shared<fm_trajectories::StripTrajectory>(settings);
}

}  // namespace fm_mission_planner
