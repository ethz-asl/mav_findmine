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

#ifndef FM_MISSION_PLANNER_STRIP_WIDGET_H_
#define FM_MISSION_PLANNER_STRIP_WIDGET_H_

#ifndef Q_MOC_RUN
#include <QDoubleSpinBox>
#include <QPushButton>
#include "fm_mission_planner/trajectory_widget.h"
#endif

namespace fm_mission_planner {

class StripWidget : public TrajectoryWidget {
 public:
  StripWidget(mav_trajectory_generation::InputConstraints input_constraints,
              QWidget* parent = 0);

 protected:
  void updateTrajectory() override;

 private:
  QDoubleSpinBox* A_east_edit_;
  QDoubleSpinBox* A_north_edit_;
  QDoubleSpinBox* delta_deg_edit_;
  QDoubleSpinBox* a_edit_;
  QDoubleSpinBox* offset_edit_;
  QPushButton* select_A_delta_button_;
  std::vector<QDoubleSpinBox*> altitudes_;
  QDoubleSpinBox* start_vertex_edit_;
  QDoubleSpinBox* num_revolutions_edit_;
};

}  // end namespace fm_mission_planner

#endif  // FM_MISSION_PLANNER_STRIP_WIDGET_H_
