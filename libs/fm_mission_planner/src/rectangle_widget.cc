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

#include "fm_mission_planner/rectangle_widget.h"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <memory>

#include <fm_trajectories/rectangle_trajectory.h>

namespace fm_mission_planner {

RectangleWidget::RectangleWidget(
    mav_trajectory_generation::InputConstraints input_constraints,
    QWidget* parent)
    : TrajectoryWidget(input_constraints, parent) {
  setWindowTitle("Rectangle Trajectory");

  // Vertex A parameters.
  A_east_edit_ = createDefaultSpinBox();
  A_north_edit_ = createDefaultSpinBox();
  delta_deg_edit_ = createDefaultAngleSpinBox();

  select_A_delta_button_ = new QPushButton(tr("Select"));
  select_A_delta_button_->setToolTip(
      tr("Select vertex A and direction of edge a."));

  // Remaining parameters.
  a_edit_ = createDefaultSpinBox();
  a_edit_->setMinimum(0.1);
  a_edit_->setValue(1.0);
  b_edit_ = createDefaultSpinBox();
  b_edit_->setMinimum(0.1);
  b_edit_->setValue(1.0);

  for (size_t i = 0; i < 4; ++i) {
    altitudes_.emplace_back(createDefaultSpinBox());
    altitudes_.back()->setValue(0.0);
  }
  altitudes_[0]->setValue(4.0);

  // TODO(rikba): Make integer spin box.
  num_edges_edit_ = createDefaultSpinBox();
  num_edges_edit_->setDecimals(0);
  num_edges_edit_->setMinimum(1);
  num_edges_edit_->setValue(4);
  num_edges_edit_->setSingleStep(1);

  start_vertex_edit_ = createDefaultSpinBox();
  start_vertex_edit_->setDecimals(0);
  start_vertex_edit_->setRange(0, 3);
  start_vertex_edit_->setSingleStep(1);
  start_vertex_edit_->setWrapping(true);

  // The overall layout.
  QGridLayout* layout = new QGridLayout;

  layout->addWidget(
      new QLabel(
          tr("Vertex A [East m, North m], direction edge a [ENU degree]:")),
      0, 0, 1, -1);

  layout->addWidget(A_east_edit_, 1, 0);
  layout->addWidget(A_north_edit_, 1, 1);
  layout->addWidget(delta_deg_edit_, 1, 2);
  layout->addWidget(select_A_delta_button_, 1, 3);

  layout->addWidget(new QLabel(tr("Edge length a [m]:")), 2, 0);
  layout->addWidget(a_edit_, 2, 1);

  layout->addWidget(new QLabel(tr("Edge length b [m]:")), 3, 0);
  layout->addWidget(b_edit_, 3, 1);

  layout->addWidget(new QLabel(tr("Absolute height A [m]:")), 4, 0);
  layout->addWidget(altitudes_[0], 4, 1);

  layout->addWidget(new QLabel(tr("Relative height B [m]:")), 5, 0);
  layout->addWidget(altitudes_[1], 5, 1);

  layout->addWidget(new QLabel(tr("Relative height C [m]:")), 6, 0);
  layout->addWidget(altitudes_[2], 6, 1);

  layout->addWidget(new QLabel(tr("Relative height D [m]:")), 7, 0);
  layout->addWidget(altitudes_[3], 7, 1);

  layout->addWidget(new QLabel(tr("Velocity [m/s]:")), 8, 0);
  layout->addWidget(velocity_edit_, 8, 1);

  layout->addWidget(new QLabel(tr("Offset heading [deg]:")), 9, 0);
  layout->addWidget(heading_offset_edit_, 9, 1);

  layout->addWidget(new QLabel(tr("Start vertex:")), 10, 0);
  layout->addWidget(start_vertex_edit_, 10, 1);

  layout->addWidget(new QLabel(tr("Number of edges to traverse:")), 11, 0);
  layout->addWidget(num_edges_edit_, 11, 1);

  layout->addWidget(confirm_button_, 12, 3);

  setLayout(layout);

  // Connections.
  // Check if trajectory draft needs to be updated.
  connectAllSpinBoxes();

  // Vertex direction selection.
  connect(select_A_delta_button_, &QPushButton::clicked, this, [this] {
    clickNavGoal(A_east_edit_, A_north_edit_, delta_deg_edit_);
  });
}

void RectangleWidget::updateTrajectory() {
  std::shared_ptr<fm_trajectories::RectangleTrajectory::Settings> settings =
      std::make_shared<fm_trajectories::RectangleTrajectory::Settings>(
          A_east_edit_->value(), A_north_edit_->value(),
          delta_deg_edit_->value(), a_edit_->value(), b_edit_->value(),
          altitudes_[0]->value(), altitudes_[1]->value(),
          altitudes_[2]->value(), altitudes_[3]->value(),
          velocity_edit_->value(), heading_offset_edit_->value(),
          prev_trajectory_, input_constraints_,
          static_cast<size_t>(start_vertex_edit_->value()),
          static_cast<size_t>(num_edges_edit_->value()));
  trajectory_ =
      std::make_shared<fm_trajectories::RectangleTrajectory>(settings);
}

}  // namespace fm_mission_planner
