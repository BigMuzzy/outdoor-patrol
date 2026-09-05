// Copyright 2026 Outdoor Patrol Team
// Licensed under the Apache License, Version 2.0.

#include "outdoor_patrol_validation/field_dashboard_panel.hpp"

#include <QFont>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <yaml-cpp/yaml.h>

namespace outdoor_patrol_validation
{

namespace
{

// Verdict colours. Chosen to read on a laptop screen in daylight, which is
// the only place this panel is ever used: saturated fills, dark text.
const QColor kPassBg("#c8f0d2");
const QColor kPassFg("#0b5f24");
const QColor kFailBg("#ffd2cd");
const QColor kFailFg("#8b1a13");
const QColor kPendBg("#ffeec2");
const QColor kPendFg("#7a5300");
const QColor kInfoBg("#e6e9ee");
const QColor kInfoFg("#41485a");
const QColor kStaleBg("#d5d7dc");
const QColor kStaleFg("#6b7280");

QColor bgFor(const std::string & status)
{
  if (status == "pass") {return kPassBg;}
  if (status == "fail") {return kFailBg;}
  if (status == "info") {return kInfoBg;}
  return kPendBg;
}

QColor fgFor(const std::string & status)
{
  if (status == "pass") {return kPassFg;}
  if (status == "fail") {return kFailFg;}
  if (status == "info") {return kInfoFg;}
  return kPendFg;
}

QString labelFor(const std::string & status)
{
  if (status == "pass") {return "PASS";}
  if (status == "fail") {return "FAIL";}
  if (status == "info") {return "";}
  return "...";
}

// -- yaml-cpp as a JSON reader ---------------------------------------------
// YAML 1.2 is a superset of JSON, so the node's json.dumps() output parses
// directly. That avoids pulling a JSON library in for one message type.

bool present(const YAML::Node & node)
{
  return node && !node.IsNull();
}

YAML::Node at(const YAML::Node & node, const char * key)
{
  if (!node || !node.IsMap()) {
    return YAML::Node(YAML::NodeType::Undefined);
  }
  return node[key];
}

std::string asString(const YAML::Node & node, const std::string & fallback = "--")
{
  if (!present(node)) {return fallback;}
  try {
    return node.as<std::string>();
  } catch (const YAML::Exception &) {
    return fallback;
  }
}

bool asBool(const YAML::Node & node, bool fallback = false)
{
  if (!present(node)) {return fallback;}
  try {
    return node.as<bool>();
  } catch (const YAML::Exception &) {
    return fallback;
  }
}

int asInt(const YAML::Node & node, int fallback = -1)
{
  if (!present(node)) {return fallback;}
  try {
    return node.as<int>();
  } catch (const YAML::Exception &) {
    return fallback;
  }
}

/// Returns false when the value is absent or null, so "--" and "0.00" stay
/// distinguishable. A missing sensor must never render as a zero.
bool asDouble(const YAML::Node & node, double & out)
{
  if (!present(node)) {return false;}
  try {
    out = node.as<double>();
    return true;
  } catch (const YAML::Exception &) {
    return false;
  }
}

QString fixed(double value, int digits, const QString & unit = "")
{
  return QString::number(value, 'f', digits) + unit;
}

}  // namespace

FieldDashboardPanel::FieldDashboardPanel(QWidget * parent)
: rviz_common::Panel(parent), last_state_time_(0, 0, RCL_ROS_TIME)
{
  buildUi();
}

FieldDashboardPanel::~FieldDashboardPanel() = default;

// -- construction ------------------------------------------------------------

Tile FieldDashboardPanel::addTile(
  ::QGridLayout * grid, int row, int column, const QString & caption)
{
  Tile tile;
  auto * box = new QWidget();
  box->setObjectName("tile");
  box->setStyleSheet(
    "QWidget#tile { background: #e6e9ee; border: 1px solid #b9bfcc;"
    " border-radius: 4px; }");
  auto * layout = new QVBoxLayout(box);
  layout->setContentsMargins(6, 4, 6, 4);
  layout->setSpacing(0);

  tile.value = new QLabel("--");
  QFont big = tile.value->font();
  big.setPointSize(std::max(15, big.pointSize() + 6));
  big.setBold(true);
  tile.value->setFont(big);
  tile.value->setAlignment(Qt::AlignCenter);

  tile.caption = new QLabel(caption);
  QFont small = tile.caption->font();
  small.setPointSize(std::max(7, small.pointSize() - 1));
  tile.caption->setFont(small);
  tile.caption->setAlignment(Qt::AlignCenter);
  tile.caption->setStyleSheet("color: #41485a;");

  layout->addWidget(tile.value);
  layout->addWidget(tile.caption);
  grid->addWidget(box, row, column);
  return tile;
}

void FieldDashboardPanel::buildUi()
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);
  root->setSpacing(6);

  auto * header = new QHBoxLayout();
  title_ = new QLabel("FIELD VALIDATION");
  QFont title_font = title_->font();
  title_font.setBold(true);
  title_font.setPointSize(std::max(11, title_font.pointSize() + 2));
  title_->setFont(title_font);
  connection_ = new QLabel("waiting for field_dashboard");
  connection_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  header->addWidget(title_);
  header->addStretch(1);
  header->addWidget(connection_);
  root->addLayout(header);

  // -- live readouts --------------------------------------------------------
  auto * tiles = new QGridLayout();
  tiles->setSpacing(4);
  sigma_ = addTile(tiles, 0, 0, "sigma (gated)");
  quality_ = addTile(tiles, 0, 1, "fix quality");
  heading_ = addTile(tiles, 0, 2, "heading ENU");
  speed_ = addTile(tiles, 0, 3, "speed");
  state_ = addTile(tiles, 1, 0, "follower");
  cross_track_ = addTile(tiles, 1, 1, "cross-track");
  offset_ = addTile(tiles, 1, 2, "d_cmd");
  clearance_ = addTile(tiles, 1, 3, "clearance F | side");
  root->addLayout(tiles);

  // -- phases ---------------------------------------------------------------
  auto * phase_box = new QGroupBox("Phases");
  auto * phase_layout = new QVBoxLayout(phase_box);
  phase_layout->setContentsMargins(6, 4, 6, 6);
  phase_list_ = new QListWidget();
  phase_list_->setAlternatingRowColors(false);
  phase_list_->setMinimumHeight(150);
  phase_list_->setSelectionMode(QAbstractItemView::SingleSelection);
  // Keep the verdict colour visible on the selected row: Qt's default
  // highlight paints over it, which hides the one thing the row is for.
  phase_list_->setStyleSheet(
    "QListWidget::item:selected { background: transparent; color: black;"
    " border: 2px solid #2b5fd9; }");
  phase_layout->addWidget(phase_list_);
  root->addWidget(phase_box);

  gate_ = new QLabel();
  gate_->setWordWrap(true);
  gate_->setStyleSheet("color: #41485a; font-style: italic;");
  root->addWidget(gate_);

  action_ = new QLabel();
  action_->setWordWrap(true);
  action_->setStyleSheet(
    "background: #eef2ff; border: 1px solid #c3ccf5; border-radius: 3px;"
    " padding: 4px; color: #26314f;");
  root->addWidget(action_);

  managed_ = new QLabel();
  managed_->setWordWrap(true);
  managed_->setStyleSheet("color: #41485a;");
  root->addWidget(managed_);

  // -- gates for the selected phase ----------------------------------------
  checks_ = new QTableWidget(0, 3);
  checks_->setHorizontalHeaderLabels({"Gate", "Value", ""});
  checks_->verticalHeader()->setVisible(false);
  checks_->setSelectionMode(QAbstractItemView::NoSelection);
  checks_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  checks_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  checks_->horizontalHeader()->setSectionResizeMode(
    1, QHeaderView::ResizeToContents);
  checks_->horizontalHeader()->setSectionResizeMode(
    2, QHeaderView::ResizeToContents);
  checks_->setMinimumHeight(150);
  root->addWidget(checks_, 1);

  hint_ = new QLabel();
  hint_->setWordWrap(true);
  hint_->setStyleSheet(
    "background: #ffd2cd; border: 1px solid #f0a79e; border-radius: 3px;"
    " padding: 4px; color: #8b1a13;");
  hint_->setVisible(false);
  root->addWidget(hint_);

  // -- live tuning ----------------------------------------------------------
  // The three knobs worth reaching for with a run in front of you. Everything
  // else stays in the profile, where it is reviewable.
  tuning_box_ = new QGroupBox("Follower (applies on next run)");
  auto * tune_layout = new QGridLayout(tuning_box_);
  tune_layout->setContentsMargins(6, 4, 6, 6);
  tune_layout->setSpacing(4);

  avoidance_ = new QCheckBox("Obstacle avoidance");
  avoidance_->setToolTip(
    "Off = track the taught line only, no retreat manoeuvres.\n"
    "Use it to measure what localization alone can do.\n"
    "The forward safety brake is NOT affected.");
  tune_layout->addWidget(avoidance_, 0, 0, 1, 2);

  show_corridor_ = new QCheckBox("Draw corridor");
  show_corridor_->setToolTip(
    "Show the lane (white) and corridor (orange) bands.\n"
    "Purely a display choice -- the green route and the\n"
    "look-ahead point stay either way.");
  tune_layout->addWidget(show_corridor_, 0, 2);

  speed_spin_ = new QDoubleSpinBox();
  speed_spin_->setRange(0.05, 2.0);
  speed_spin_->setSingleStep(0.05);
  speed_spin_->setDecimals(2);
  speed_spin_->setSuffix(" m/s");
  tune_layout->addWidget(new QLabel("speed"), 1, 0);
  tune_layout->addWidget(speed_spin_, 1, 1);

  corridor_spin_ = new QDoubleSpinBox();
  corridor_spin_->setRange(0.0, 6.0);
  corridor_spin_->setSingleStep(0.1);
  corridor_spin_->setDecimals(2);
  corridor_spin_->setSuffix(" m");
  corridor_spin_->setToolTip(
    "Half-width the follower may use to get around an obstacle.\n"
    "Keep it below the route's tightest corner radius or the\n"
    "offset lane folds -- the follower will refuse and say so.");
  tune_layout->addWidget(new QLabel("corridor"), 2, 0);
  tune_layout->addWidget(corridor_spin_, 2, 1);

  apply_ = new QPushButton("Apply");
  tune_layout->addWidget(apply_, 1, 2, 2, 1);
  root->addWidget(tuning_box_);

  // -- controls -------------------------------------------------------------
  auto * buttons = new QGridLayout();
  buttons->setSpacing(4);
  start_ = new QPushButton("Start");
  stop_ = new QPushButton("Stop");
  pass_ = new QPushButton("Mark pass");
  fail_ = new QPushButton("Mark fail");
  reset_ = new QPushButton("Reset");
  report_ = new QPushButton("Write report");
  start_->setStyleSheet("font-weight: bold;");
  pass_->setStyleSheet("color: #0b5f24;");
  fail_->setStyleSheet("color: #8b1a13;");
  buttons->addWidget(start_, 0, 0);
  buttons->addWidget(stop_, 0, 1);
  buttons->addWidget(reset_, 0, 2);
  buttons->addWidget(pass_, 1, 0);
  buttons->addWidget(fail_, 1, 1);
  buttons->addWidget(report_, 1, 2);
  root->addLayout(buttons);

  log_ = new QTextEdit();
  log_->setReadOnly(true);
  log_->setMaximumHeight(90);
  QFont mono("monospace");
  mono.setStyleHint(QFont::TypeWriter);
  mono.setPointSize(std::max(7, log_->font().pointSize() - 1));
  log_->setFont(mono);
  root->addWidget(log_);

  connect(start_, &QPushButton::clicked, this, &FieldDashboardPanel::onStart);
  connect(stop_, &QPushButton::clicked, this, &FieldDashboardPanel::onStop);
  connect(pass_, &QPushButton::clicked, this, &FieldDashboardPanel::onPass);
  connect(fail_, &QPushButton::clicked, this, &FieldDashboardPanel::onFail);
  connect(reset_, &QPushButton::clicked, this, &FieldDashboardPanel::onReset);
  connect(report_, &QPushButton::clicked, this, &FieldDashboardPanel::onReport);
  connect(
    phase_list_, &QListWidget::currentRowChanged,
    this, &FieldDashboardPanel::onPhaseSelected);
  connect(
    avoidance_, &QCheckBox::toggled,
    this, &FieldDashboardPanel::onAvoidanceToggled);
  connect(
    show_corridor_, &QCheckBox::toggled,
    this, &FieldDashboardPanel::onShowCorridorToggled);
  connect(apply_, &QPushButton::clicked,
    this, &FieldDashboardPanel::onApplyTuning);

  setConnected(false);
}

void FieldDashboardPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    state_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&FieldDashboardPanel::onState, this, std::placeholders::_1));
  command_pub_ = node_->create_publisher<std_msgs::msg::String>(
    command_topic_, rclcpp::QoS(10).reliable());

  // RViz owns the executor, so the subscription fires on its thread. Qt work
  // happens here, at a rate a human can read rather than the 5 Hz it arrives.
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &FieldDashboardPanel::refresh);
  timer_->start(200);
}

void FieldDashboardPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("StateTopic", &topic) && !topic.isEmpty()) {
    state_topic_ = topic.toStdString();
  }
  if (config.mapGetString("CommandTopic", &topic) && !topic.isEmpty()) {
    command_topic_ = topic.toStdString();
  }
}

void FieldDashboardPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("StateTopic", QString::fromStdString(state_topic_));
  config.mapSetValue("CommandTopic", QString::fromStdString(command_topic_));
}

// -- ROS ---------------------------------------------------------------------

void FieldDashboardPanel::onState(const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_state_ = msg->data;
  have_state_ = true;
  last_state_time_ = node_->now();
}

void FieldDashboardPanel::sendCommand(
  const std::string & action, int phase, const std::string & verdict)
{
  if (!command_pub_) {return;}
  std::string json = "{\"action\": \"" + action + "\"";
  if (phase >= 0) {
    json += ", \"phase\": " + std::to_string(phase);
  }
  if (!verdict.empty()) {
    json += ", \"verdict\": \"" + verdict + "\"";
  }
  json += "}";
  std_msgs::msg::String msg;
  msg.data = json;
  command_pub_->publish(msg);
}

void FieldDashboardPanel::onStart() {sendCommand("start", selected_phase_, "");}
void FieldDashboardPanel::onStop() {sendCommand("stop", -1, "");}
void FieldDashboardPanel::onPass() {sendCommand("mark", selected_phase_, "pass");}
void FieldDashboardPanel::onFail() {sendCommand("mark", selected_phase_, "fail");}
void FieldDashboardPanel::onReset() {sendCommand("reset", selected_phase_, "");}
void FieldDashboardPanel::onReport() {sendCommand("report", -1, "");}

void FieldDashboardPanel::sendParams(const std::string & json_body)
{
  if (!command_pub_) {return;}
  std_msgs::msg::String msg;
  msg.data = "{\"action\": \"set_param\", \"params\": {" + json_body + "}}";
  command_pub_->publish(msg);
}

void FieldDashboardPanel::onAvoidanceToggled(bool enabled)
{
  // Toggling is deliberately immediate rather than waiting for Apply: it is
  // the one control you reach for because something is happening now.
  if (updating_tuning_) {return;}
  sendParams(std::string("\"avoidance_enabled\": ") +
    (enabled ? "true" : "false"));
}

void FieldDashboardPanel::onShowCorridorToggled(bool shown)
{
  if (updating_tuning_) {return;}
  sendParams(std::string("\"show_corridor\": ") + (shown ? "true" : "false"));
}

void FieldDashboardPanel::onApplyTuning()
{
  std::string body = "\"nominal_speed_ms\": " +
    QString::number(speed_spin_->value(), 'f', 3).toStdString();
  body += ", \"corridor_half_width_m\": " +
    QString::number(corridor_spin_->value(), 'f', 3).toStdString();
  sendParams(body);
}

void FieldDashboardPanel::onPhaseSelected(int row)
{
  if (row < 0 || updating_list_) {return;}
  selected_phase_ = row;
  // Redraw immediately rather than waiting up to 200 ms for the next tick.
  rendered_state_.clear();
}

// -- rendering ---------------------------------------------------------------

void FieldDashboardPanel::setConnected(bool connected)
{
  connection_->setText(connected ? "connected" : "NO DASHBOARD NODE");
  connection_->setStyleSheet(
    connected ? "color: #0b5f24; font-weight: bold;"
    : "color: #8b1a13; font-weight: bold;");
  for (QPushButton * button : {start_, stop_, pass_, fail_, reset_, report_}) {
    if (button) {button->setEnabled(connected);}
  }
  if (stop_) {stop_->setEnabled(connected && active_phase_ >= 0);}
}

void FieldDashboardPanel::refresh()
{
  std::string json;
  bool have = false;
  double age = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    json = latest_state_;
    have = have_state_;
    if (have && node_) {
      age = (node_->now() - last_state_time_).seconds();
    }
  }

  // The node publishes at 5 Hz; a second of silence means it is gone, and a
  // frozen dashboard showing a stale sigma is worse than no dashboard.
  const bool connected = have && age < 2.0;
  setConnected(connected);
  if (!connected) {
    for (Tile * tile :
      {&sigma_, &quality_, &heading_, &speed_, &state_, &cross_track_,
        &offset_, &clearance_})
    {
      tile->value->setStyleSheet("color: #6b7280;");
      tile->value->setText("--");
    }
    // Force a full repaint on reconnect. Without this, a node that restarts
    // and republishes an identical document would be skipped as unchanged,
    // leaving the tiles blank under a green "connected".
    rendered_state_.clear();
    return;
  }
  if (json == rendered_state_) {return;}
  rendered_state_ = json;
  applyState(json);
}

void FieldDashboardPanel::applyState(const std::string & json)
{
  YAML::Node root;
  try {
    root = YAML::Load(json);
  } catch (const YAML::Exception &) {
    return;
  }
  if (!root || !root.IsMap()) {return;}

  const YAML::Node sig = at(root, "signals");  // not `signals`: Qt #defines it
  active_phase_ = asInt(at(root, "active"), -1);

  auto paint = [](Tile & tile, const QString & text, const QColor & colour) {
      tile.value->setText(text);
      tile.value->setStyleSheet("color: " + colour.name() + ";");
    };

  // sigma: the number the whole go/no-go turns on. Thresholds mirror
  // confidence_gate (5 cm) and route_alley's sigma_stop_m (15 cm).
  double value = 0.0;
  if (asBool(at(sig, "gated_ok")) && asDouble(at(sig, "sigma_gated"), value)) {
    const QColor colour = value <= 0.05 ? kPassFg : (value <= 0.15 ? kPendFg : kFailFg);
    paint(sigma_, fixed(value * 100.0, 1, " cm"), colour);
    sigma_.caption->setText("sigma (gated)");
  } else {
    paint(sigma_, "--", kStaleFg);
    sigma_.caption->setText("sigma  NO FIX");
  }

  const int quality = asInt(at(sig, "gga_quality"), -1);
  const std::string fix_class = asString(at(sig, "fix_class"), "");
  if (asBool(at(sig, "gga_ok")) && quality >= 0) {
    const QColor colour = quality == 4 ? kPassFg : (quality == 5 ? kPendFg : kFailFg);
    paint(quality_, QString::fromStdString(asString(at(sig, "gga_quality_name"))),
      colour);
    QString caption = QString("%1 sats").arg(asInt(at(sig, "gga_sats"), 0));
    double age = 0.0;
    if (asDouble(at(sig, "gga_corr_age"), age)) {
      caption += "  age " + fixed(age, 1, " s");
    }
    const std::string station = asString(at(sig, "gga_station"), "");
    if (!station.empty()) {
      caption += "  base " + QString::fromStdString(station);
    }
    quality_.caption->setText(caption);
  } else if (asBool(at(sig, "raw_ok")) && !fix_class.empty()) {
    // No NMEA -- the Gazebo sim synthesises a NavSatFix and publishes no raw
    // sentences. Fall back to the class derived from status and sigma, which
    // is what route_recorder classifies on anyway. Say where it came from, so
    // a missing NMEA stream on the real robot is still visible.
    const QColor colour = fix_class == "fixed" ? kPassFg
      : (fix_class == "float" ? kPendFg : kFailFg);
    paint(quality_, QString::fromStdString(fix_class), colour);
    quality_.caption->setText("from covariance (no NMEA)");
  } else {
    paint(quality_, "--", kStaleFg);
    quality_.caption->setText("no GGA");
  }

  if (asBool(at(sig, "heading_ok")) &&
    asDouble(at(sig, "heading_yaw_deg"), value))
  {
    paint(heading_, fixed(value, 1, QString::fromUtf8("\u00b0")), kInfoFg);
  } else {
    paint(heading_, "--", kStaleFg);
  }

  if (asDouble(at(sig, "follower_speed"), value) ||
    asDouble(at(sig, "odom_speed"), value))
  {
    paint(speed_, fixed(value, 2, " m/s"), kInfoFg);
  } else {
    paint(speed_, "--", kStaleFg);
  }

  if (asBool(at(sig, "follower_ok"))) {
    const std::string state = asString(at(sig, "state"), "--");
    QColor colour = kInfoFg;
    if (state == "driving" || state == "finished") {colour = kPassFg;}
    if (state == "retreating" || state == "resuming") {colour = kPendFg;}
    if (state == "blocked" || state == "degraded") {colour = kFailFg;}
    paint(state_, QString::fromStdString(state), colour);
  } else {
    paint(state_, "idle", kStaleFg);
  }

  if (asDouble(at(sig, "cross_track"), value)) {
    const double magnitude = std::abs(value);
    const QColor colour =
      magnitude <= 0.2 ? kPassFg : (magnitude <= 0.5 ? kPendFg : kFailFg);
    paint(cross_track_, fixed(value, 2, " m"), colour);
  } else {
    paint(cross_track_, "--", kStaleFg);
  }

  if (asDouble(at(sig, "d_cmd"), value)) {
    // Positive is the wall the obstacle is against: it is a failure on sight,
    // not a value to read carefully.
    const QColor colour = value > 0.001 ? kFailFg : kInfoFg;
    paint(offset_, QString(value >= 0 ? "+" : "") + fixed(value, 2, " m"), colour);
  } else {
    paint(offset_, "--", kStaleFg);
  }

  double front = 0.0;
  double left = 0.0;
  double right = 0.0;
  const bool have_front = asDouble(at(sig, "scan_front_min"), front);
  const bool have_left = asDouble(at(sig, "scan_left_min"), left);
  const bool have_right = asDouble(at(sig, "scan_right_min"), right);
  const bool scan_alive = asBool(at(sig, "scan_ok"));
  if (scan_alive && (have_front || have_left || have_right)) {
    const double side = std::min(
      have_left ? left : 99.0, have_right ? right : 99.0);
    const QColor colour =
      (have_front && front < 0.6) ? kFailFg : (side < 1.0 ? kPendFg : kPassFg);
    paint(
      clearance_,
      (have_front ? fixed(front, 1) : QString("--")) + " | " +
      (side < 99.0 ? fixed(side, 1, " m") : QString("--")), colour);
    clearance_.caption->setText("clearance F | side");
  } else if (scan_alive) {
    // The lidar is publishing, there is simply nothing within range -- an
    // open road rather than a dead sensor. Reporting this as NO SCAN would
    // send an operator hunting a lidar fault that does not exist.
    paint(clearance_, "clear", kPassFg);
    clearance_.caption->setText("nothing within range");
  } else {
    paint(clearance_, "--", kStaleFg);
    clearance_.caption->setText("clearance  NO SCAN");
  }

  // -- live tuning ----------------------------------------------------------
  // ALWAYS editable. The corridor width and the avoidance switch are what you
  // decide while setting a run UP, and route_follower does not exist yet to
  // receive them -- so the dashboard owns them and hands them over when it
  // launches one. Gating these on a running follower made them unreachable at
  // exactly the moment they were wanted.
  const YAML::Node tuning = at(root, "tuning");
  const bool follower_live = asBool(at(tuning, "live"));
  const YAML::Node desired = at(tuning, "desired");
  if (present(desired)) {
    updating_tuning_ = true;
    // Checkboxes track the dashboard every cycle: you cannot be "mid-edit"
    // on a tickbox, so following it keeps them honest when the value is
    // changed from a terminal.
    avoidance_->setChecked(asBool(at(desired, "avoidance_enabled"), true));
    show_corridor_->setChecked(asBool(at(desired, "show_corridor"), true));
    // Spin boxes are seeded ONCE. Following them would overwrite a
    // half-typed number five times a second.
    if (!tuning_seeded_) {
      double v = 0.0;
      if (asDouble(at(desired, "nominal_speed_ms"), v)) {
        speed_spin_->setValue(v);
      }
      if (asDouble(at(desired, "corridor_half_width_m"), v)) {
        corridor_spin_->setValue(v);
      }
      tuning_seeded_ = true;
    }
    updating_tuning_ = false;
  }
  if (tuning_box_ != nullptr) {
    tuning_box_->setTitle(
      follower_live ? "Follower (live)" : "Follower (applies on next run)");
  }

  // -- phase list -----------------------------------------------------------
  const YAML::Node phases = at(root, "phases");
  const int count = (phases && phases.IsSequence()) ?
    static_cast<int>(phases.size()) : 0;

  if (phase_list_->count() != count) {
    updating_list_ = true;
    phase_list_->clear();
    for (int i = 0; i < count; ++i) {
      phase_list_->addItem(new QListWidgetItem());
    }
    updating_list_ = false;
  }

  QFont row_font = phase_list_->font();
  row_font.setBold(true);
  for (int i = 0; i < count; ++i) {
    const YAML::Node phase = phases[i];
    const std::string verdict = asString(at(phase, "verdict"), "pending");
    const std::string name = asString(at(phase, "name"), "?");
    const bool running = (i == active_phase_);
    const bool manual = asBool(at(phase, "manual"));

    QString text = QString("%1%2  %3")
      .arg(running ? QString::fromUtf8("\u25b6 ") : QString("   "))
      .arg(i)
      .arg(QString::fromStdString(name), -16);
    text += QString("  %1%2")
      .arg(labelFor(verdict))
      .arg(manual ? " (manual)" : "");

    QListWidgetItem * item = phase_list_->item(i);
    item->setText(text);
    item->setBackground(bgFor(verdict));
    item->setForeground(fgFor(verdict));
    item->setFont(row_font);
  }

  // Pressing Start is a request to watch that phase, so follow the active one
  // whenever it changes. Between changes the operator's own selection stands.
  updating_list_ = true;
  if (active_phase_ >= 0 && active_phase_ != last_active_phase_) {
    selected_phase_ = active_phase_;
    phase_list_->setCurrentRow(active_phase_);
  }
  if (phase_list_->currentRow() < 0 && count > 0) {
    phase_list_->setCurrentRow(std::min(selected_phase_, count - 1));
  }
  updating_list_ = false;
  last_active_phase_ = active_phase_;
  selected_phase_ = std::max(0, std::min(selected_phase_, count - 1));

  // -- gates for the selected phase -----------------------------------------
  if (selected_phase_ >= count) {return;}
  const YAML::Node phase = phases[selected_phase_];
  gate_->setText(
    QString("Gate: %1").arg(QString::fromStdString(asString(at(phase, "gate"), ""))));
  action_->setText(QString::fromStdString(asString(at(phase, "action"), "")));

  // What the dashboard itself has running. Without this the operator cannot
  // tell "I pressed Start and it is recording" from "I pressed Start and
  // nothing happened", which is exactly the confusion that lost a teach pass.
  const YAML::Node managed = at(root, "managed");
  QString mtext;
  if (present(managed)) {
    const bool rec = asBool(at(managed, "recorder"));
    const bool fol = asBool(at(managed, "follower"));
    const std::string route = asString(at(managed, "route"), "");
    mtext = QString("recorder: %1    follower: %2")
      .arg(rec ? "RUNNING" : (asBool(at(managed, "manage_recorder"))
        ? "idle" : "manual"))
      .arg(fol ? "RUNNING" : (asBool(at(managed, "manage_follower"))
        ? "idle" : "manual"));
    mtext += QString("    route: %1")
      .arg(route.empty() ? QString("none recorded yet")
        : QString::fromStdString(route));
    managed_->setStyleSheet(
      (rec || fol) ? "color: #0b5f24; font-weight: bold;" : "color: #41485a;");
  }
  managed_->setText(mtext);

  const YAML::Node checks = at(phase, "checks");
  const int rows = (checks && checks.IsSequence()) ?
    static_cast<int>(checks.size()) : 0;
  checks_->setRowCount(rows);

  QString first_hint;
  for (int i = 0; i < rows; ++i) {
    const YAML::Node check = checks[i];
    const std::string status = asString(at(check, "status"), "pending");
    const std::string hint = asString(at(check, "hint"), "");
    if (status == "fail" && first_hint.isEmpty() && !hint.empty()) {
      first_hint = QString::fromStdString(hint);
    }

    const QString cells[3] = {
      QString::fromStdString(asString(at(check, "label"), "")),
      QString::fromStdString(asString(at(check, "value"), "")),
      labelFor(status),
    };
    for (int column = 0; column < 3; ++column) {
      QTableWidgetItem * item = checks_->item(i, column);
      if (item == nullptr) {
        item = new QTableWidgetItem();
        checks_->setItem(i, column, item);
      }
      item->setText(cells[column]);
      item->setBackground(bgFor(status));
      item->setForeground(fgFor(status));
      if (!hint.empty()) {
        item->setToolTip(QString::fromStdString(hint));
      }
      if (column == 2) {
        QFont font = item->font();
        font.setBold(true);
        item->setFont(font);
      }
    }
  }
  checks_->resizeRowsToContents();

  hint_->setText(first_hint);
  hint_->setVisible(!first_hint.isEmpty());

  // -- log ------------------------------------------------------------------
  const YAML::Node log = at(root, "log");
  QString text;
  if (log && log.IsSequence()) {
    for (const auto & line : log) {
      text += QString::fromStdString(asString(line, "")) + "\n";
    }
  }
  if (log_->toPlainText() != text) {
    log_->setPlainText(text);
    log_->moveCursor(QTextCursor::End);
  }
}

}  // namespace outdoor_patrol_validation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  outdoor_patrol_validation::FieldDashboardPanel, rviz_common::Panel)
