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

#ifndef FM_MISSION_PLANNER_MISSION_PLANNER_PANEL_H_
#define FM_MISSION_PLANNER_MISSION_PLANNER_PANEL_H_

#ifndef Q_MOC_RUN
#include <list>
#include <memory>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/NavSatFix.h>
#include <QComboBox>
#include <QLineEdit>
#include <QMetaEnum>
#include <QPushButton>
#include <QString>
#include <QTextEdit>

#include <fm_trajectories/base_trajectory.h>
#include "fm_mission_planner/coverage_widget.h"
#include "fm_mission_planner/hotpoint_widget.h"
#include "fm_mission_planner/mission.h"
#include "fm_mission_planner/rectangle_widget.h"
#include "fm_mission_planner/strip_widget.h"
#endif

namespace fm_mission_planner {

class MissionPlannerPanel : public rviz::Panel {
  Q_OBJECT

 public:
  MissionPlannerPanel(QWidget* parent = 0);
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;
  void onInitialize() override;

  enum TrajectoryType { Hotpoint = 0, Rectangle, Strip, Coverage };
  Q_ENUM(TrajectoryType)

 public Q_SLOTS:

  // Add a trajectory.
  void addTrajectory();
  void receiveTrajectory(
      const std::shared_ptr<fm_trajectories::BaseTrajectory>& trajectory);
  // Save the trajectory to a file.
  void saveToFile();
  // Export the polynomial trajectory.
  void exportToFile();

 private Q_SLOTS:
  // Constantly publish trajectory.
  void visualizeTrajectories();

  // Set the trajectory type when selection changes.
  void setTrajectoryType(const QString& type);
  void enablePanel();
  void clearMission();

 private:
  void geodeticReferenceCallback(const sensor_msgs::NavSatFixConstPtr& msg);
  void getRosParameters();

  QPushButton* save_button_;
  QPushButton* add_button_;
  QPushButton* clear_button_;
  QTextEdit* trajectory_info_;
  QComboBox* trajectory_type_box_;
  QPushButton* export_button_;

  HotpointWidget* hotpoint_widget_;
  RectangleWidget* rectangle_widget_;
  StripWidget* strip_widget_;
  CoverageWidget* coverage_widget_;

  TrajectoryType trajectory_type_;
  Mission mission_;

  // The ROS publisher for the trajectory visualization.
  ros::Publisher trajectory_publisher_;
  ros::Subscriber geo_ref_sub_;

  // The ROS node handle.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
};

}  // end namespace fm_mission_planner

#endif  // FM_MISSION_PLANNER_MISSION_PLANNER_PANEL_H_
