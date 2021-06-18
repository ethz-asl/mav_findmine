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

#ifndef FM_MISSION_PLANNER_HOTPOINT_WIDGET_H_
#define FM_MISSION_PLANNER_HOTPOINT_WIDGET_H_

#ifndef Q_MOC_RUN
#include <QDoubleSpinBox>
#include <QPushButton>
#include "fm_mission_planner/trajectory_widget.h"
#endif

namespace fm_mission_planner {

class HotpointWidget : public TrajectoryWidget {
 public:
  HotpointWidget(mav_trajectory_generation::InputConstraints input_constraints,
                 QWidget* parent = 0);

 protected:
  void updateTrajectory() override;

 private:
  QDoubleSpinBox* east_edit_;
  QDoubleSpinBox* north_edit_;
  QDoubleSpinBox* altitude_edit_;
  QDoubleSpinBox* radius_edit_;
  QDoubleSpinBox* start_heading_deg_edit_;
  QDoubleSpinBox* arc_length_deg_edit_;
  QDoubleSpinBox* circle_deviation_ratio_edit_;
  QPushButton* select_center_button_;
};

}  // end namespace fm_mission_planner

#endif  // FM_MISSION_PLANNER_HOTPOINT_WIDGET_H_
