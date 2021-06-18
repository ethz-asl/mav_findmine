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

#ifndef FM_MISSION_PLANNER_TRAJECTORY_WIDGET_H_
#define FM_MISSION_PLANNER_TRAJECTORY_WIDGET_H_

#ifndef Q_MOC_RUN
#include <fm_trajectories/base_trajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_trajectory_generation_ros/input_constraints.h>
#include <ros/ros.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <QCheckBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <memory>
#endif

namespace fm_mission_planner {

class TrajectoryWidget : public QDialog {
  Q_OBJECT
 public:
  TrajectoryWidget(
      mav_trajectory_generation::InputConstraints input_constraints,
      QWidget* parent = 0);
  void setVisualizationManager(rviz::VisualizationManager* vis_manager);
  void setPreviousTrajectory(
      const std::weak_ptr<fm_trajectories::BaseTrajectory>& prev_trajectory);

 public Q_SLOTS:
  void checkSpinBoxes();
  void reject() override;

 protected Q_SLOTS:
  void clickPoint(QDoubleSpinBox* click_x, QDoubleSpinBox* click_y);
  void clickNavGoal(QDoubleSpinBox* click_x, QDoubleSpinBox* click_y,
                    QDoubleSpinBox* click_delta);
  void receivedClickedPoint();

 private Q_SLOTS:
  void sendTrajectory();

 Q_SIGNALS:
  void outputTrajectory(
      const std::shared_ptr<fm_trajectories::BaseTrajectory>& trajectory);

 protected:
  virtual void updateTrajectory() = 0;
  void connectAllSpinBoxes();
  QDoubleSpinBox* createDefaultSpinBox();
  QDoubleSpinBox* createDefaultAngleSpinBox();

  QPushButton* confirm_button_;
  std::shared_ptr<fm_trajectories::BaseTrajectory> trajectory_;
  std::weak_ptr<fm_trajectories::BaseTrajectory> prev_trajectory_;

  // Common parameters.
  QDoubleSpinBox* velocity_edit_;
  QDoubleSpinBox* heading_offset_edit_;
  mav_trajectory_generation::InputConstraints input_constraints_;
  QLabel* trajectory_time_edit_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher trajectory_draft_pub_;
  // children need to access this class so make it protected
  // acces is needed to acces the tool manager that launches
  // the polygon tool
  rviz::VisualizationManager* vis_manager_;

 private:
  void clickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void navGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::Subscriber clicked_point_sub_;
  ros::Subscriber nav_goal_sub_;
  QDoubleSpinBox* click_x_;
  QDoubleSpinBox* click_y_;
  QDoubleSpinBox* click_delta_;
  bool expecting_clicked_point_;
  bool expecting_2d_nav_goal_;
};

}  // end namespace fm_mission_planner

#endif  // FM_MISSION_PLANNER_TRAJECTORY_WIDGET_H_
