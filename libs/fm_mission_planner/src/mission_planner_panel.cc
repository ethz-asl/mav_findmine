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

#include <QByteArray>
#include <QComboBox>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QIODevice>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QTextEdit>
#include <QTextStream>
#include <QVBoxLayout>

#include <glog/logging.h>

#include "fm_mission_planner/hotpoint_widget.h"
#include "fm_mission_planner/mission_planner_panel.h"

namespace mtg = mav_trajectory_generation;

namespace fm_mission_planner {

const QString kInitTrajectoryInfo = "'Add...' a new trajectory!";

MissionPlannerPanel::MissionPlannerPanel(QWidget* parent)
    : rviz::Panel(parent),
      trajectory_type_(TrajectoryType::Hotpoint),
      nh_private_("~") {
  // ROS stuff.
  geo_ref_sub_ =
      nh_.subscribe("geo_ref", 1,
                    &MissionPlannerPanel::geodeticReferenceCallback, this);
  trajectory_publisher_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("mission", 1);
  getRosParameters();

  // Save and clear buttons.
  save_button_ = new QPushButton(tr("Save..."));
  save_button_->setToolTip(tr("Save mission to a file."));
  save_button_->setEnabled(false);

  clear_button_ = new QPushButton(tr("Clear..."));
  clear_button_->setToolTip(tr("Clear mission."));
  clear_button_->setEnabled(false);

  QHBoxLayout* save_file_layout = new QHBoxLayout;
  save_file_layout->addWidget(save_button_);
  save_file_layout->addWidget(clear_button_);

  // Add trajectory widget.
  // Trajectory type combo box.
  // https://stackoverflow.com/questions/33215182/qmetaenum-and-strong-typed-enum
  const QMetaObject& mo = MissionPlannerPanel::staticMetaObject;
  int index = mo.indexOfEnumerator("TrajectoryType");
  QMetaEnum meta_enum = mo.enumerator(index);

  QComboBox* trajectory_type_box_ = new QComboBox;
  trajectory_type_box_->addItem(
      meta_enum.valueToKey(static_cast<int>(TrajectoryType::Hotpoint)));
  trajectory_type_box_->addItem(
      meta_enum.valueToKey(static_cast<int>(TrajectoryType::Rectangle)));
  trajectory_type_box_->addItem(
      meta_enum.valueToKey(static_cast<int>(TrajectoryType::Strip)));
  trajectory_type_box_->addItem(
      meta_enum.valueToKey(static_cast<int>(TrajectoryType::Coverage)));
  int init_type = trajectory_type_box_->findText(
      meta_enum.valueToKey(static_cast<int>(trajectory_type_)));
  CHECK_NE(init_type, -1);
  trajectory_type_box_->setCurrentIndex(init_type);

  add_button_ = new QPushButton(tr("Add..."));
  add_button_->setToolTip(tr("Add a trajectory to the mission."));

  QHBoxLayout* trajectory_type_layout = new QHBoxLayout;
  trajectory_type_layout->addWidget(trajectory_type_box_);
  trajectory_type_layout->addWidget(add_button_);

  // The trajectory edit windows.
  hotpoint_widget_ = new HotpointWidget(mission_.getInputConstraints(), 0);
  rectangle_widget_ = new RectangleWidget(mission_.getInputConstraints(), 0);
  strip_widget_ = new StripWidget(mission_.getInputConstraints(), 0);
  coverage_widget_ = new CoverageWidget(mission_.getInputConstraints(), 0);
  // Trajectory info.
  trajectory_info_ = new QTextEdit(kInitTrajectoryInfo);
  trajectory_info_->setReadOnly(true);

  // Export trajectory.
  export_button_ = new QPushButton(tr("Export..."));
  export_button_->setToolTip(
      tr("Export the trajectory to a UAV readable file."));

  // Connections.
  connect(save_button_, SIGNAL(clicked()), this, SLOT(saveToFile()));
  connect(clear_button_, SIGNAL(clicked()), this, SLOT(clearMission()));
  connect(add_button_, SIGNAL(clicked()), this, SLOT(addTrajectory()));
  connect(trajectory_type_box_, SIGNAL(currentIndexChanged(QString)), this,
          SLOT(setTrajectoryType(QString)));
  connect(export_button_, SIGNAL(clicked()), this, SLOT(exportToFile()));
  // Active panel when dialog is over.
  connect(hotpoint_widget_, SIGNAL(accepted()), this, SLOT(enablePanel()));
  connect(hotpoint_widget_, SIGNAL(rejected()), this, SLOT(enablePanel()));
  connect(hotpoint_widget_,
          SIGNAL(outputTrajectory(
              const std::shared_ptr<fm_trajectories::BaseTrajectory>&)),
          this,
          SLOT(receiveTrajectory(
              const std::shared_ptr<fm_trajectories::BaseTrajectory>&)));

  connect(rectangle_widget_, SIGNAL(accepted()), this, SLOT(enablePanel()));
  connect(rectangle_widget_, SIGNAL(rejected()), this, SLOT(enablePanel()));
  connect(rectangle_widget_,
          SIGNAL(outputTrajectory(
              const std::shared_ptr<fm_trajectories::BaseTrajectory>&)),
          this,
          SLOT(receiveTrajectory(
              const std::shared_ptr<fm_trajectories::BaseTrajectory>&)));

  connect(strip_widget_, SIGNAL(accepted()), this, SLOT(enablePanel()));
  connect(strip_widget_, SIGNAL(rejected()), this, SLOT(enablePanel()));
  connect(strip_widget_,
          SIGNAL(outputTrajectory(
              const std::shared_ptr<fm_trajectories::BaseTrajectory>&)),
          this,
          SLOT(receiveTrajectory(
              const std::shared_ptr<fm_trajectories::BaseTrajectory>&)));

  connect(coverage_widget_, SIGNAL(accepted()), this, SLOT(enablePanel()));
  connect(coverage_widget_, SIGNAL(rejected()), this, SLOT(enablePanel()));
  connect(coverage_widget_,
          SIGNAL(outputTrajectory(
              const std::shared_ptr<fm_trajectories::BaseTrajectory>&)),
          this,
          SLOT(receiveTrajectory(
              const std::shared_ptr<fm_trajectories::BaseTrajectory>&)));

  // Initialize the overall layout.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(save_file_layout);
  layout->addLayout(trajectory_type_layout);
  layout->addWidget(trajectory_info_);
  layout->addWidget(export_button_);
  setLayout(layout);
}

void MissionPlannerPanel::geodeticReferenceCallback(
    const sensor_msgs::NavSatFixConstPtr& msg) {
  mission_.setGeodeticReference(msg->latitude, msg->longitude, msg->altitude);
}

void MissionPlannerPanel::getRosParameters() {
  // Input constraints.
  mav_trajectory_generation::InputConstraints input_constraints;
  input_constraints.setDefaultValues();
  double f_min;
  if (nh_private_.getParam("input_constraints/f_min", f_min))
    input_constraints.addConstraint(mtg::InputConstraintType::kFMin, f_min);
  double f_max;
  if (nh_private_.getParam("input_constraints/f_max", f_max))
    input_constraints.addConstraint(mtg::InputConstraintType::kFMax, f_max);
  double v_max;
  if (nh_private_.getParam("input_constraints/v_max", v_max))
    input_constraints.addConstraint(mtg::InputConstraintType::kVMax, v_max);
  double omega_xy_max;
  if (nh_private_.getParam("input_constraints/omega_xy_max", omega_xy_max))
    input_constraints.addConstraint(mtg::InputConstraintType::kOmegaXYMax,
                                    omega_xy_max);
  double omega_z_max;
  if (nh_private_.getParam("input_constraints/omega_z_max", omega_z_max))
    input_constraints.addConstraint(mtg::InputConstraintType::kOmegaZMax,
                                    omega_z_max);
  double omega_z_dot_max;
  if (nh_private_.getParam("input_constraints/omega_z_dot_max",
                           omega_z_dot_max))
    input_constraints.addConstraint(mtg::InputConstraintType::kOmegaZDotMax,
                                    omega_z_dot_max);
  mission_.setInputConstraints(input_constraints);
}

void MissionPlannerPanel::enablePanel() { setEnabled(true); }

void MissionPlannerPanel::saveToFile() {
  QString file_name =
      QFileDialog::getSaveFileName(this, tr("Save Mission"), QDir::tempPath(),
                                   tr("Mission (*.yaml);;All Files (*)"));

  if (file_name.isEmpty())
    return;
  else {
    QFile file(file_name);
    if (!file.open(QIODevice::WriteOnly)) {
      QMessageBox::information(this, tr("Unable to open file."),
                               file.errorString());
      return;
    }

    QTextStream out(&file);
    out << QString::fromStdString(mission_.toString());
  }
}

void MissionPlannerPanel::exportToFile() {
  QString file_name =
      QFileDialog::getSaveFileName(this, tr("Export Mission"), QDir::tempPath(),
                                   tr("Trajectory (*.yaml);;All Files (*)"));

  if (file_name.isEmpty())
    return;
  else {
    QFile file(file_name);
    if (!file.open(QIODevice::WriteOnly)) {
      QMessageBox::information(this, tr("Unable to open file."),
                               file.errorString());
      return;
    }

    QTextStream out(&file);
    out << QString::fromStdString(mission_.exportTrajectory());
  }
}

void MissionPlannerPanel::setTrajectoryType(const QString& type) {
  const QMetaObject& mo = MissionPlannerPanel::staticMetaObject;
  int index = mo.indexOfEnumerator("TrajectoryType");
  QMetaEnum meta_enum = mo.enumerator(index);

  QByteArray ba = type.toLocal8Bit();
  const char* key = ba.data();

  trajectory_type_ = static_cast<TrajectoryType>(meta_enum.keyToValue(key));
}

void MissionPlannerPanel::addTrajectory() {
  switch (trajectory_type_) {
    case TrajectoryType::Hotpoint: {
      hotpoint_widget_->show();
      hotpoint_widget_->checkSpinBoxes();
      break;
    }
    case TrajectoryType::Rectangle: {
      rectangle_widget_->show();
      rectangle_widget_->checkSpinBoxes();
      break;
    }
    case TrajectoryType::Strip: {
      strip_widget_->show();
      strip_widget_->checkSpinBoxes();
      break;
    }
    case TrajectoryType::Coverage: {
      coverage_widget_->show();
      break;
    }
    default: {
      ROS_WARN("Requested trajectory type not available.");
      break;
    }
  }
  setEnabled(false);
}

void MissionPlannerPanel::receiveTrajectory(
    const std::shared_ptr<fm_trajectories::BaseTrajectory>& trajectory) {
  ROS_INFO_STREAM("Receiving trajectory.");
  CHECK_NOTNULL(trajectory);

  trajectory_publisher_.publish(mission_.getTrajectoryDeleteMarkers());
  mission_.addTrajectory(trajectory);

  // Set previous trajectory.
  hotpoint_widget_->setPreviousTrajectory(trajectory);
  rectangle_widget_->setPreviousTrajectory(trajectory);
  strip_widget_->setPreviousTrajectory(trajectory);
  coverage_widget_->setPreviousTrajectory(trajectory);

  trajectory_publisher_.publish(mission_.getTrajectoryMarkers());
  trajectory_info_->setText(QString::fromStdString(mission_.toString()));
  save_button_->setEnabled(true);
  clear_button_->setEnabled(true);
}

// Save rviz config.
void MissionPlannerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

// Load rviz config.
void MissionPlannerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

void MissionPlannerPanel::visualizeTrajectories() {}

void MissionPlannerPanel::onInitialize() {
  hotpoint_widget_->setVisualizationManager(vis_manager_);
  rectangle_widget_->setVisualizationManager(vis_manager_);
  strip_widget_->setVisualizationManager(vis_manager_);
  coverage_widget_->setVisualizationManager(vis_manager_);
}

void MissionPlannerPanel::clearMission() {
  trajectory_publisher_.publish(mission_.getTrajectoryDeleteMarkers());
  mission_.clear();
  trajectory_info_->setText(kInitTrajectoryInfo);
  save_button_->setEnabled(false);
  clear_button_->setEnabled(false);
}
}  // end namespace fm_mission_planner

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.h>
#include <rviz/panel.h>
PLUGINLIB_EXPORT_CLASS(fm_mission_planner::MissionPlannerPanel, rviz::Panel)
