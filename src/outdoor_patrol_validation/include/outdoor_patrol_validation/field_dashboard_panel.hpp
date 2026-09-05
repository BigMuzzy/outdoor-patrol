// Copyright 2026 Outdoor Patrol Team
// Licensed under the Apache License, Version 2.0.
//
// One-screen field validation dashboard, as an RViz2 panel.
//
// Dock it beside the 3D view and the whole alley procedure fits in a single
// window: the route, corridor and lidar in the scene, and every number and
// gate from doc/eng/plans/field-validation-alley.md in the panel. That is the
// point of building it as a panel rather than a separate Qt app -- outdoors,
// on a laptop, alt-tabbing between a visualiser and a dashboard while a robot
// is moving is exactly the failure mode this replaces.
//
// The panel holds NO gate logic. It renders whatever `field_dashboard`
// publishes on its state topic and sends button presses back on the command
// topic. Everything that decides pass or fail lives in the Python node, where
// it is unit-tested and where it keeps running if RViz is closed.

#ifndef OUTDOOR_PATROL_VALIDATION__FIELD_DASHBOARD_PANEL_HPP_
#define OUTDOOR_PATROL_VALIDATION__FIELD_DASHBOARD_PANEL_HPP_

#include <QColor>
#include <QString>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

class QLabel;
class QListWidget;
class QGridLayout;
class QPushButton;
class QCheckBox;
class QGroupBox;
class QDoubleSpinBox;
class QTableWidget;
class QTextEdit;
class QTimer;

namespace outdoor_patrol_validation
{

/// A single large number in the readout strip.
struct Tile
{
  QLabel * value = nullptr;
  QLabel * caption = nullptr;
};

class FieldDashboardPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit FieldDashboardPanel(QWidget * parent = nullptr);
  ~FieldDashboardPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  /// Repaint from the latest state message. GUI thread; see refresh().
  void refresh();
  void onStart();
  void onStop();
  void onPass();
  void onFail();
  void onReset();
  void onReport();
  void onPhaseSelected(int row);
  /// Live tuning, pushed to route_follower through the dashboard node.
  void onAvoidanceToggled(bool enabled);
  void onShowCorridorToggled(bool shown);
  void onApplyTuning();

private:
  void buildUi();
  Tile addTile(::QGridLayout * grid, int row, int column,
    const QString & caption);
  void sendCommand(const std::string & action, int phase,
    const std::string & verdict);
  /// {"action":"set_param","params":{...}} -- JSON body built by the caller.
  void sendParams(const std::string & json_body);
  void onState(const std_msgs::msg::String::ConstSharedPtr msg);
  void applyState(const std::string & json);
  void setConnected(bool connected);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;

  // The ROS callback runs on RViz's executor thread, Qt repaints on the GUI
  // thread. The string is the only thing that crosses, under this mutex.
  std::mutex mutex_;
  std::string latest_state_;
  std::string rendered_state_;
  bool have_state_ = false;
  rclcpp::Time last_state_time_;

  std::string state_topic_ = "/field_dashboard/state";
  std::string command_topic_ = "/field_dashboard/command";

  QTimer * timer_ = nullptr;
  QLabel * title_ = nullptr;
  QLabel * connection_ = nullptr;
  QLabel * gate_ = nullptr;
  QLabel * action_ = nullptr;
  /// Managed-node state: what the dashboard has running, and which route.
  QLabel * managed_ = nullptr;
  /// What to do about the first failing gate, quoted from the plan.
  QLabel * hint_ = nullptr;
  QListWidget * phase_list_ = nullptr;
  QTableWidget * checks_ = nullptr;
  QTextEdit * log_ = nullptr;
  QPushButton * start_ = nullptr;
  QPushButton * stop_ = nullptr;
  QPushButton * pass_ = nullptr;
  QPushButton * fail_ = nullptr;
  QPushButton * reset_ = nullptr;
  QPushButton * report_ = nullptr;
  QGroupBox * tuning_box_ = nullptr;
  QCheckBox * avoidance_ = nullptr;
  QCheckBox * show_corridor_ = nullptr;
  QDoubleSpinBox * speed_spin_ = nullptr;
  QDoubleSpinBox * corridor_spin_ = nullptr;
  QPushButton * apply_ = nullptr;
  /// True while code is writing the widgets, so the resulting signals are
  /// not mistaken for the operator turning a knob.
  bool updating_tuning_ = false;
  /// Seed the spin boxes from the follower once, then leave them alone --
  /// otherwise a half-typed value is overwritten 5 times a second.
  bool tuning_seeded_ = false;

  Tile sigma_;
  Tile quality_;
  Tile heading_;
  Tile state_;
  Tile cross_track_;
  Tile offset_;
  Tile clearance_;
  Tile speed_;

  int selected_phase_ = 0;
  int active_phase_ = -1;
  int last_active_phase_ = -1;
  /// True while the code is driving the phase list, so the resulting
  /// currentRowChanged is not mistaken for an operator click.
  bool updating_list_ = false;
};

}  // namespace outdoor_patrol_validation

#endif  // OUTDOOR_PATROL_VALIDATION__FIELD_DASHBOARD_PANEL_HPP_
