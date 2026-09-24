/** @file
 *
 * Simple rqt plugin to edit trajectories.
 *
 * @author Jan Razlaw
 */

#include <rviz2_cinematographer_gui/rviz2_cinematographer_gui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QHeaderView>
#include <QItemSelection>
#include <QMetaObject>
#include <QVariant>
#include <QTableWidget>

namespace rviz2_cinematographer_gui
{

// tf2::fromMsg for quaternions returns void - this is a value returning convenience wrapper
static inline tf2::Quaternion quaternionFromMsg(const geometry_msgs::msg::Quaternion& msg)
{
  tf2::Quaternion quat;
  tf2::fromMsg(msg, quat);
  return quat;
}

/// Slerps between two orientations.
static inline geometry_msgs::msg::Quaternion slerp(const geometry_msgs::msg::Quaternion& start,
                                                   const geometry_msgs::msg::Quaternion& end,
                                                   double factor)
{
  return tf2::toMsg(quaternionFromMsg(start).slerp(quaternionFromMsg(end), factor));
}

static inline std::vector<std::string> splitOnWhitespace(const std::string& line)
{
  std::vector<std::string> tokens;
  std::istringstream stream(line);
  std::string token;
  while(stream >> token)
    tokens.push_back(token);
  return tokens;
}

static inline bool startsWith(const std::string& text, const std::string& prefix)
{
  return text.compare(0, prefix.size(), prefix) == 0;
}

RvizCinematographerGUI::RvizCinematographerGUI()
  : rqt_gui_cpp::Plugin()
    , widget_(0)
    , current_marker_name_("")
{
  //cam_pose_.orientation.w = 1.0;

  // give QObjects reasonable names
  setObjectName("RvizCinematographerGUI");
}

void RvizCinematographerGUI::postToGuiThread(std::function<void()> function)
{
  QMetaObject::invokeMethod(this, std::move(function), Qt::QueuedConnection);
}

std::function<void(const RvizCinematographerGUI::FeedbackConstPtr&)> RvizCinematographerGUI::inGuiThread(
  std::function<void(const FeedbackConstPtr&)> callback)
{
  return [this, callback](const FeedbackConstPtr& feedback)
  {
    postToGuiThread([callback, feedback]() { callback(feedback); });
  };
}

void RvizCinematographerGUI::initPlugin(qt_gui_cpp::PluginContext& context)
{
  camera_trajectory_pub_ = node_->create_publisher<rviz2_cinematographer_msgs::msg::CameraTrajectory>("/rviz/camera_trajectory", rclcpp::QoS(1));
  view_poses_array_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/transformed_path", rclcpp::QoS(1).transient_local());
  record_params_pub_ = node_->create_publisher<rviz2_cinematographer_msgs::msg::Record>("/rviz/record", rclcpp::QoS(1));

  // plugin arguments - passed to rqt behind "--args"
  // supported: --trajectory-file <path_to_yaml>   --start-recorder <true|false>
  const QStringList argv = context.argv();
  std::string trajectory_file;
  bool start_recorder = true;
  for(int i = 0; i + 1 < argv.size(); ++i)
  {
    if(argv[i] == "--trajectory-file")
      trajectory_file = argv[++i].toStdString();
    else if(argv[i] == "--start-recorder")
      start_recorder = QVariant(argv[++i]).toBool();
  }

  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  qRegisterMetaType<QItemSelection>();

  connect(ui_.add_before_push_button, SIGNAL(clicked(bool)), this, SLOT(addMarkerBefore()));
  connect(ui_.add_here_push_button, SIGNAL(clicked(bool)), this, SLOT(addMarkerHere()));
  connect(ui_.add_after_push_button, SIGNAL(clicked(bool)), this, SLOT(addMarkerBehind()));
  connect(ui_.delete_push_button, SIGNAL(clicked(bool)), this, SLOT(removeCurrentMarker()));

  connect(ui_.translation_x_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.translation_y_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.translation_z_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));

  connect(ui_.rotation_x_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.rotation_y_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.rotation_z_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));
  connect(ui_.rotation_w_spin_box, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentMarker()));

  connect(ui_.move_to_current_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToCurrent()));
  connect(ui_.move_to_prev_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToPrev()));
  connect(ui_.move_to_first_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToFirst()));
  connect(ui_.move_to_next_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToNext()));
  connect(ui_.move_to_last_button, SIGNAL(clicked(bool)), this, SLOT(moveCamToLast()));

  connect(ui_.append_cam_pose, SIGNAL(clicked(bool)), this, SLOT(appendCamPoseToTrajectory()));
  connect(ui_.set_pose_to_cam_button, SIGNAL(clicked(bool)), this, SLOT(setCurrentPoseToCam()));
  connect(ui_.frame_line_edit, SIGNAL(editingFinished()), this, SLOT(setMarkerFrames()));
  connect(ui_.splines_check_box, SIGNAL(stateChanged(int)), this, SLOT(updateTrajectory()));
  connect(ui_.marker_size_increase, SIGNAL(clicked(bool)), this, SLOT(increaseMarkerScale()));
  connect(ui_.marker_size_decrease, SIGNAL(clicked(bool)), this, SLOT(decreaseMarkerScale()));
  connect(ui_.show_interactive_marker_controls_check_box, SIGNAL(stateChanged(int)), this, SLOT(showInteractiveMarkerControls()));

  connect(ui_.video_output_path_tool_button, SIGNAL(clicked(bool)), this, SLOT(setVideoOutputPath()));

  connect(ui_.open_file_push_button, SIGNAL(clicked(bool)), this, SLOT(loadTrajectoryFromFile()));
  connect(ui_.save_file_push_button, SIGNAL(clicked(bool)), this, SLOT(saveTrajectoryToFile()));

  // add widget to the user interface
  context.addWidget(widget_);

  menu_handler_.insert("Add marker before", inGuiThread(std::bind(&RvizCinematographerGUI::addMarkerBeforeClicked, this, std::placeholders::_1)));
  menu_handler_.insert("Add marker here", inGuiThread(std::bind(&RvizCinematographerGUI::addMarkerAtClicked, this, std::placeholders::_1)));
  menu_handler_.insert("Add marker after", inGuiThread(std::bind(&RvizCinematographerGUI::addMarkerBehindClicked, this, std::placeholders::_1)));
  menu_handler_.insert("Remove marker", inGuiThread(std::bind(&RvizCinematographerGUI::removeClickedMarker, this, std::placeholders::_1)));

  // set up markers
  bool markers_loaded = false;
  if(!trajectory_file.empty())
  {
    markers_loaded = loadTrajectoryFromYaml(trajectory_file);
    if(!markers_loaded)
      RCLCPP_ERROR_STREAM(getLogger(), "Could not load trajectory from file: " << trajectory_file);
  }

  if(!markers_loaded)
  {
    markers_.clear();
    visualization_msgs::msg::InteractiveMarker marker_0 = makeMarker();
    marker_0.name = "1";
    marker_0.description = "1";
    marker_0.controls[0].markers[0].color.g = 1.f;
    markers_.emplace_back(TimedMarker(std::move(marker_0), 2.5));
    visualization_msgs::msg::InteractiveMarker marker_1 = makeMarker(2.0, 0.0, 1.0);
    marker_1.name = "2";
    marker_1.description = "2";
    marker_1.controls[0].markers[0].color.r = 1.f;
    markers_.emplace_back(TimedMarker(std::move(marker_1), 2.5));
  }

  setCurrentTo(markers_.front());

  // connect markers to callback functions
  // only MOUSE_UP events are of interest - filter before handing them to the GUI thread
  auto process_feedback = inGuiThread(std::bind(&RvizCinematographerGUI::processFeedback, this, std::placeholders::_1));
  marker_feedback_callback_ = [process_feedback](const FeedbackConstPtr& feedback)
  {
    if(feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
      process_feedback(feedback);
  };
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("trajectory", node_);
  updateServer(markers_);

  setUpTimeTable();

  updateTrajectory();

  camera_pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
    "/rviz/current_camera_pose", rclcpp::QoS(1),
    std::bind(&RvizCinematographerGUI::camPoseCallback, this, std::placeholders::_1));
  record_finished_sub_ = node_->create_subscription<rviz2_cinematographer_msgs::msg::Finished>(
    "/video_recorder/record_finished", rclcpp::QoS(1),
    std::bind(&RvizCinematographerGUI::recordFinishedCallback, this, std::placeholders::_1));
  delete_marker_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
    "/rviz/delete", rclcpp::QoS(1),
    std::bind(static_cast<void (RvizCinematographerGUI::*)(const std_msgs::msg::Empty::ConstSharedPtr)>(
                &RvizCinematographerGUI::removeCurrentMarker), this, std::placeholders::_1));

  if(start_recorder)
    startVideoRecorder();
  else
    RCLCPP_WARN(getLogger(), "Video recorder was not started.");
}

void RvizCinematographerGUI::startVideoRecorder()
{
  if(video_recorder_executor_)
    return;

  video_recorder_node_ = std::make_shared<rviz2_cinematographer_video_recorder::VideoRecorder>(rclcpp::NodeOptions());
  video_recorder_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  video_recorder_executor_->add_node(video_recorder_node_);
  video_recorder_thread_ = std::thread([this]() { video_recorder_executor_->spin(); });
}

void RvizCinematographerGUI::stopVideoRecorder()
{
  if(!video_recorder_executor_)
    return;

  video_recorder_executor_->cancel();
  video_recorder_thread_.join();
  video_recorder_executor_->remove_node(video_recorder_node_);
  video_recorder_executor_.reset();
  video_recorder_node_.reset();
}

void setUpTableHeader(QTableWidget* marker_table_widget)
{
  marker_table_widget->setColumnCount(2);
  QStringList table_header;
  table_header << "Transition Duration" << "Wait Duration";
  marker_table_widget->setHorizontalHeaderLabels(table_header);
  QTableWidgetItem* headerItem = marker_table_widget->horizontalHeaderItem(0);
  if (headerItem)
    headerItem->setToolTip("The duration in seconds needed to move the camera from the previous to this marker.\n"
                           "Beware that the transition duration of the marker the trajectory is starting from is neglected.");
  headerItem = marker_table_widget->horizontalHeaderItem(1);
  if(headerItem)
    headerItem->setToolTip("Wait for specified duration in seconds after reaching this marker.");
}

void deleteTableContents(QTableWidget* marker_table_widget)
{
  for(int row = 0; row < marker_table_widget->rowCount(); row++)
  {
    for(int col = 0; col < 2; col++)
    {
      marker_table_widget->removeCellWidget(row, col);
    }
  }

  marker_table_widget->setRowCount(0);
}

void RvizCinematographerGUI::refillTable()
{
  deleteTableContents(ui_.marker_table_widget);

  for(const auto& marker : markers_)
  {
    int row = ui_.marker_table_widget->rowCount();
    ui_.marker_table_widget->insertRow(row);

    std::vector<double> durations = {marker.transition_duration, marker.wait_duration};
    for(int i = 0; i < 2; i++)
    {
      ui_.marker_table_widget->setCellWidget(row, i, new QDoubleSpinBox(ui_.marker_table_widget));
      auto q = qobject_cast<QDoubleSpinBox*>(ui_.marker_table_widget->cellWidget(row, i));
      if(q)
      {
        q->setAlignment(Qt::AlignmentFlag::AlignRight);
        q->setSuffix(" seconds");
        q->setDecimals(1);
        q->setMinimum(0.0);
        q->setMaximum(999.0);
        q->setSingleStep(0.1);
        q->setValue(durations[i]);

        q->setProperty("row", row);
        q->setProperty("column", i);
        connect(q, SIGNAL(valueChanged(double)), this, SLOT(updateMarker()));
      }
    }
  }

  auto first_transition_duration_spin_box = qobject_cast<QDoubleSpinBox*>(ui_.marker_table_widget->cellWidget(0, 0));
  if(first_transition_duration_spin_box)
    first_transition_duration_spin_box->setEnabled(false);

  ui_.marker_table_widget->resizeColumnsToContents();
  ui_.marker_table_widget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  ui_.marker_table_widget->horizontalHeader()->setStretchLastSection(true);

  disconnect(ui_.marker_table_widget->verticalHeader(), SIGNAL(sectionClicked(int)), nullptr, nullptr);
  ui_.marker_table_widget->selectRow(getMarkerId(current_marker_name_));
  connect(ui_.marker_table_widget->verticalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(updateWhoIsCurrentMarker(int)));
}

void RvizCinematographerGUI::setUpTimeTable()
{
  setUpTableHeader(ui_.marker_table_widget);
  refillTable();
}

void RvizCinematographerGUI::shutdownPlugin()
{
  // create empty path to "erase" previous path on shutdown
  nav_msgs::msg::Path path;
  if(!markers_.empty())
    path.header = markers_.front().marker.header;

  markers_.clear();
  if(server_)
  {
    server_->clear();
    server_->applyChanges();
  }

  camera_pose_sub_.reset();
  record_finished_sub_.reset();
  delete_marker_sub_.reset();
  camera_trajectory_pub_.reset();
  record_params_pub_.reset();

  if(view_poses_array_pub_)
  {
    view_poses_array_pub_->publish(path);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // give the publisher some time
    view_poses_array_pub_.reset();
  }

  stopVideoRecorder();

  server_.reset();
}

void RvizCinematographerGUI::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                          qt_gui_cpp::Settings& instance_settings) const
{
  (void)plugin_settings;
  (void)instance_settings;
  // TODO save intrinsic configuration, usually using:
  //instance_settings.setValue(k, v)
}

void RvizCinematographerGUI::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                             const qt_gui_cpp::Settings& instance_settings)
{
  (void)plugin_settings;
  (void)instance_settings;
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void RvizCinematographerGUI::camPoseCallback(const geometry_msgs::msg::Pose::ConstSharedPtr cam_pose)
{
  postToGuiThread([this, cam_pose]() { cam_pose_ = *cam_pose; });
}

void RvizCinematographerGUI::updateTrajectory()
{
  if(markers_.size() < 2)
    return;

  nav_msgs::msg::Path path;
  path.header = markers_.front().marker.header;

  if(ui_.splines_check_box->isChecked())
  {
    std::vector<geometry_msgs::msg::Pose> spline_poses;
    markersToSplinedPoses(markers_, spline_poses, ui_.publish_rate_spin_box->value());
    for(auto& pose : spline_poses)
    {
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.pose = pose;
      waypoint.header = path.header;
      path.poses.push_back(waypoint);
    }
  }
  else
  {
    for(const auto& marker : markers_)
    {
      visualization_msgs::msg::InteractiveMarker int_marker;
      if(!server_->get(marker.marker.name, int_marker))
        int_marker = marker.marker;

      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.pose = int_marker.pose;
      waypoint.header = path.header;
      path.poses.push_back(waypoint);
    }
  }

  view_poses_array_pub_->publish(path);

  server_->applyChanges();
}

void RvizCinematographerGUI::safeTrajectoryToFile(const std::string& file_path)
{
  std::ofstream file;
  file.open(file_path, std::ofstream::trunc);
  file << "rviz_cinematographer_camera_poses:\n";
  for(const auto& marker : markers_)
  {
    file << std::fixed << std::setprecision(6);
    file << "  -\n";
    file << "    position:\n";
    file << "      x: " << marker.marker.pose.position.x << "\n";
    file << "      y: " << marker.marker.pose.position.y << "\n";
    file << "      z: " << marker.marker.pose.position.z << "\n";
    file << "    orientation:\n";
    file << "      x: " << marker.marker.pose.orientation.x << "\n";
    file << "      y: " << marker.marker.pose.orientation.y << "\n";
    file << "      z: " << marker.marker.pose.orientation.z << "\n";
    file << "      w: " << marker.marker.pose.orientation.w << "\n";
    file << "    transition_duration: " << marker.transition_duration << "\n";
    file << "    wait_duration: " << marker.wait_duration << "\n";
  }
  file.close();
}

void RvizCinematographerGUI::addMarkerBefore()
{
  addMarkerBefore(current_marker_name_);
}

void RvizCinematographerGUI::addMarkerBeforeClicked(const FeedbackConstPtr& feedback)
{
  setCurrentFromTo(getMarkerByName(current_marker_name_), getMarkerByName(feedback->marker_name));
  addMarkerBefore();
}

void RvizCinematographerGUI::addMarkerBefore(const std::string& current_marker_name)
{
  auto clicked_element = findMarker(current_marker_name);
  if(clicked_element == markers_.end())
    return;

  colorizeMarkersRed();

  // initialize new marker between clicked and previous - or right beside clicked if first marker selected
  visualization_msgs::msg::InteractiveMarker new_marker = clicked_element->marker;
  new_marker.controls[0].markers[0].color.r = 0.f;
  new_marker.controls[0].markers[0].color.g = 1.f;
  if(clicked_element != markers_.begin())
  {
    const geometry_msgs::msg::Pose& pose_before = std::prev(clicked_element)->marker.pose;
    const geometry_msgs::msg::Pose& clicked_pose = clicked_element->marker.pose;
    new_marker.pose.position.x = (pose_before.position.x + clicked_pose.position.x) / 2.;
    new_marker.pose.position.y = (pose_before.position.y + clicked_pose.position.y) / 2.;
    new_marker.pose.position.z = (pose_before.position.z + clicked_pose.position.z) / 2.;
    new_marker.pose.orientation = slerp(pose_before.orientation, clicked_pose.orientation, 0.5);
  }
  else
  {
    new_marker.pose.position.x -= 0.5;
  }
  clicked_element = markers_.insert(clicked_element,
                                    TimedMarker(std::move(new_marker), clicked_element->transition_duration, clicked_element->wait_duration));

  current_marker_name_ = current_marker_name;

  // update server with updated member markers
  server_->clear();
  updateServer(markers_);

  refillTable();

  updateGUIValues(*clicked_element);
  updateTrajectory();
}

void RvizCinematographerGUI::addMarkerHere()
{
  addMarkerHere(current_marker_name_);
}

void RvizCinematographerGUI::addMarkerAtClicked(const FeedbackConstPtr& feedback)
{
  setCurrentFromTo(getMarkerByName(current_marker_name_), getMarkerByName(feedback->marker_name));
  addMarkerHere();
}

void RvizCinematographerGUI::addMarkerHere(const std::string& current_marker_name)
{
  auto clicked_element = findMarker(current_marker_name);
  if(clicked_element == markers_.end())
    return;

  colorizeMarkersRed();

  // initialize new marker at the position of the clicked marker
  visualization_msgs::msg::InteractiveMarker new_marker = clicked_element->marker;
  new_marker.controls[0].markers[0].color.r = 0.f;
  new_marker.controls[0].markers[0].color.g = 1.f;

  markers_.insert(clicked_element, TimedMarker(std::move(new_marker), clicked_element->transition_duration, clicked_element->wait_duration));

  current_marker_name_ = current_marker_name;

  // update server with updated member markers
  server_->clear();
  updateServer(markers_);

  refillTable();

  updateGUIValues(*clicked_element);
  updateTrajectory();
}

void RvizCinematographerGUI::addMarkerBehind()
{
  addMarkerBehind(current_marker_name_);
}

void RvizCinematographerGUI::addMarkerBehindClicked(const FeedbackConstPtr& feedback)
{
  setCurrentFromTo(getMarkerByName(current_marker_name_), getMarkerByName(feedback->marker_name));
  addMarkerBehind();
}

void RvizCinematographerGUI::addMarkerBehind(const std::string& current_marker_name)
{
  auto clicked_element = findMarker(current_marker_name);
  if(clicked_element == markers_.end())
    return;

  colorizeMarkersRed();

  // initialize new marker between clicked and next marker - or right beside the clicked if last marker selected
  visualization_msgs::msg::InteractiveMarker new_marker = clicked_element->marker;
  new_marker.controls[0].markers[0].color.r = 0.f;
  new_marker.controls[0].markers[0].color.g = 1.f;
  auto element_behind = std::next(clicked_element);
  if(element_behind != markers_.end())
  {
    const geometry_msgs::msg::Pose& clicked_pose = clicked_element->marker.pose;
    const geometry_msgs::msg::Pose& pose_behind = element_behind->marker.pose;
    new_marker.pose.position.x = (clicked_pose.position.x + pose_behind.position.x) / 2.;
    new_marker.pose.position.y = (clicked_pose.position.y + pose_behind.position.y) / 2.;
    new_marker.pose.position.z = (clicked_pose.position.z + pose_behind.position.z) / 2.;
    new_marker.pose.orientation = slerp(clicked_pose.orientation, pose_behind.orientation, 0.5);
  }
  else
  {
    new_marker.pose.position.x -= 0.5;
  }
  clicked_element = markers_.insert(element_behind,
                                    TimedMarker(std::move(new_marker), clicked_element->transition_duration, clicked_element->wait_duration));

  // name of the new marker will be the one of the clicked marker incremented by 1
  current_marker_name_ = std::to_string(std::stoi(current_marker_name) + 1);

  // update server with updated member markers
  server_->clear();
  updateServer(markers_);

  refillTable();

  updateGUIValues(*clicked_element);
  updateTrajectory();
}

void RvizCinematographerGUI::setCurrentTo(TimedMarker& marker)
{
  marker.marker.controls[0].markers[0].color.r = 0.f;
  marker.marker.controls[0].markers[0].color.g = 1.f;
  current_marker_name_ = marker.marker.name;

  updateGUIValues(marker);
}

void RvizCinematographerGUI::setCurrentFromTo(TimedMarker& old_current,
                                              TimedMarker& new_current)
{
  // update current marker
  current_marker_name_ = new_current.marker.name;

  updateGUIValues(new_current);

  // update member list
  new_current.marker.controls[0].markers[0].color.r = 0.f;
  new_current.marker.controls[0].markers[0].color.g = 1.f;
  old_current.marker.controls[0].markers[0].color.r = 1.f;
  old_current.marker.controls[0].markers[0].color.g = 0.f;

  server_->clear();
  updateServer(markers_);
}

void RvizCinematographerGUI::removeCurrentMarker()
{
  removeMarker(current_marker_name_);
}

void RvizCinematographerGUI::removeCurrentMarker(const std_msgs::msg::Empty::ConstSharedPtr empty)
{
  (void)empty;
  postToGuiThread([this]() { removeCurrentMarker(); });
}

void RvizCinematographerGUI::removeClickedMarker(const FeedbackConstPtr& feedback)
{
  setCurrentFromTo(getMarkerByName(current_marker_name_), getMarkerByName(feedback->marker_name));
  removeCurrentMarker();
}

void RvizCinematographerGUI::removeMarker(const std::string& marker_name)
{
  if(markers_.size() < 3)
  {
    RCLCPP_ERROR(getLogger(), "Cannot remove last two markers.");
    return;
  }

  auto searched_element = findMarker(marker_name);

  colorizeMarkersRed();

  // set previous marker as current
  if(searched_element != markers_.end())
  {
    // if first marker is removed, replace current by second marker
    if(searched_element == markers_.begin())
    {
      setCurrentTo(*(std::next(searched_element)));
      // second element becomes first and gets first's name - all following numbered continuously
      current_marker_name_ = markers_.front().marker.name;
    }
    else
      setCurrentTo(*(std::prev(searched_element)));

    // delete selected marker from member markers
    markers_.erase(searched_element);
  }

  server_->clear();
  updateServer(markers_);

  refillTable();
  updateGUIValues(getMarkerByName(current_marker_name_));
  updateTrajectory();
}

void RvizCinematographerGUI::updateServer(MarkerList& markers)
{
  size_t count = 0;
  for(auto& marker : markers)
  {
    marker.marker.name = std::to_string(count + 1);
    marker.marker.description = std::to_string(count + 1);
    count++;
    server_->insert(marker.marker, marker_feedback_callback_);
    menu_handler_.apply(*server_, marker.marker.name);
  }

  server_->applyChanges();
}

bool RvizCinematographerGUI::loadTrajectoryFromYaml(const std::string& file_path)
{
  YAML::Node trajectory;
  try
  {
    trajectory = YAML::LoadFile(file_path);
  }
  catch(const YAML::Exception& e)
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Could not load yaml file " << file_path << " : " << e.what());
    return false;
  }

  if(!trajectory["rviz_cinematographer_camera_poses"])
  {
    RCLCPP_ERROR_STREAM(getLogger(), "File " << file_path << " does not contain the key rviz_cinematographer_camera_poses.");
    return false;
  }

  MarkerList markers;
  int count = 0;
  try
  {
    for(const auto& pose : trajectory["rviz_cinematographer_camera_poses"])
    {
      visualization_msgs::msg::InteractiveMarker wp_marker = makeMarker();
      wp_marker.controls[0].markers[0].color.r = 1.f;

      wp_marker.pose.orientation.w = pose["orientation"]["w"].as<double>();
      wp_marker.pose.orientation.x = pose["orientation"]["x"].as<double>();
      wp_marker.pose.orientation.y = pose["orientation"]["y"].as<double>();
      wp_marker.pose.orientation.z = pose["orientation"]["z"].as<double>();

      wp_marker.pose.position.x = pose["position"]["x"].as<double>();
      wp_marker.pose.position.y = pose["position"]["y"].as<double>();
      wp_marker.pose.position.z = pose["position"]["z"].as<double>();

      wp_marker.name = std::to_string(count + 1);
      wp_marker.description = std::to_string(count + 1);

      double transition_duration = pose["transition_duration"] ? pose["transition_duration"].as<double>() : 0.0;
      double wait_duration = pose["wait_duration"] ? pose["wait_duration"].as<double>() : 0.0;

      markers.emplace_back(TimedMarker(std::move(wp_marker), transition_duration, wait_duration));
      count++;
    }
  }
  catch(const YAML::Exception& e)
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Could not parse yaml file " << file_path << " : " << e.what());
    return false;
  }

  if(markers.empty())
    return false;

  markers_ = std::move(markers);
  return true;
}

bool RvizCinematographerGUI::loadTrajectoryFromTxt(const std::string& file_path)
{
  std::ifstream infile(file_path);
  if(!infile.is_open())
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Could not open file " << file_path);
    return false;
  }

  MarkerList markers;
  int count = 0;
  std::string line;
  double prev_pose_duration = 0.0;
  while(std::getline(infile, line))
  {
    if(startsWith(line, "#"))
      continue;

    std::vector<std::string> pose_strings = splitOnWhitespace(line);

    if(pose_strings.empty())
      continue;

    if(pose_strings.size() != 8)
    {
      RCLCPP_ERROR_STREAM(getLogger(), "Line: " << line
                                                << " contains the wrong number of parameters. Format is: timestamp tx ty tz qx qy qz qw. Number of parameters are "
                                                << (int)pose_strings.size());
      continue;
    }

    std::vector<double> values;
    try
    {
      for(const auto& pose_string : pose_strings)
        values.push_back(std::stod(pose_string));
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(getLogger(), "Line: " << line << " contains a value that is not a number: " << e.what());
      continue;
    }

    visualization_msgs::msg::InteractiveMarker wp_marker = makeMarker();
    wp_marker.controls[0].markers[0].color.r = 1.f;

    double transition_duration = 0.0;
    if(count > 0)
      transition_duration = values.at(0) - prev_pose_duration;

    wp_marker.pose.position.x = values.at(1);
    wp_marker.pose.position.y = values.at(2);
    wp_marker.pose.position.z = values.at(3);

    wp_marker.pose.orientation.x = values.at(4);
    wp_marker.pose.orientation.y = values.at(5);
    wp_marker.pose.orientation.z = values.at(6);
    wp_marker.pose.orientation.w = values.at(7);

    wp_marker.name = std::to_string(count + 1);
    wp_marker.description = std::to_string(count + 1);

    markers.emplace_back(TimedMarker(std::move(wp_marker), transition_duration));

    prev_pose_duration = values.at(0);
    count++;
  }

  if(markers.empty())
    return false;

  markers_ = std::move(markers);
  return true;
}

RvizCinematographerGUI::MarkerIterator RvizCinematographerGUI::findMarker(const std::string& marker_name)
{
  return std::find_if(markers_.begin(), markers_.end(),
                      [&marker_name](const TimedMarker& marker) { return marker.marker.name == marker_name; });
}

RvizCinematographerGUI::TimedMarker& RvizCinematographerGUI::getMarkerByName(const std::string& marker_name)
{
  auto marker = findMarker(marker_name);
  if(marker != markers_.end())
    return *marker;

  static TimedMarker tmp = TimedMarker(visualization_msgs::msg::InteractiveMarker(), 0.5);
  return tmp;
}

bool RvizCinematographerGUI::isCamWithinBounds()
{
  ui_.messages_label->setText(QString("Message: Right click on markers for options."));

  if(ui_.translation_x_spin_box->maximum() < cam_pose_.position.x ||
     ui_.translation_x_spin_box->minimum() > cam_pose_.position.x ||
     ui_.translation_y_spin_box->maximum() < cam_pose_.position.y ||
     ui_.translation_y_spin_box->minimum() > cam_pose_.position.y ||
     ui_.translation_z_spin_box->maximum() < cam_pose_.position.z ||
     ui_.translation_z_spin_box->minimum() > cam_pose_.position.z)
  {
    ui_.messages_label->setText(
      QString("Message: Current position is out of scope.\n\tTry moving closer to the center of the frame."));
    return false;
  }
  return true;
}

void RvizCinematographerGUI::rvizCamToMarkerOrientation(const geometry_msgs::msg::Pose& rviz_cam_pose,
                                                        geometry_msgs::msg::Pose& marker_pose)
{
  // rotate cam pose around z axis for -90 degrees
  tf2::Quaternion rot_around_z_neg_90_deg(0.0, 0.0, -0.707, 0.707);
  marker_pose.orientation = tf2::toMsg(quaternionFromMsg(rviz_cam_pose.orientation) * rot_around_z_neg_90_deg);
  marker_pose.position = rviz_cam_pose.position;
}

void RvizCinematographerGUI::appendCamPoseToTrajectory()
{
  if(!isCamWithinBounds())
    return;

  // rotate cam pose around z axis for -90 degrees
  geometry_msgs::msg::Pose rotated_cam_pose;
  rvizCamToMarkerOrientation(cam_pose_, rotated_cam_pose);

  // create new marker
  visualization_msgs::msg::InteractiveMarker new_marker = makeMarker();
  new_marker.pose.orientation.y = 0.0;
  new_marker.name = std::to_string((int)markers_.size() + 1);
  new_marker.description = std::to_string((int)markers_.size() + 1);

  // set cam pose as marker pose
  new_marker.pose = rotated_cam_pose;

  markers_.emplace_back(TimedMarker(std::move(new_marker), 0.5));

  colorizeMarkersRed();

  // set new marker as current marker
  setCurrentTo(markers_.back());

  updateServer(markers_);
  refillTable();
  updateTrajectory();
}

void RvizCinematographerGUI::setCurrentPoseToCam()
{
  if(!isCamWithinBounds())
    return;

  // rotate cam pose around z axis for -90 degrees
  geometry_msgs::msg::Pose rotated_cam_pose;
  rvizCamToMarkerOrientation(cam_pose_, rotated_cam_pose);

  // update marker pose
  getMarkerByName(current_marker_name_).marker.pose = rotated_cam_pose;
  server_->setPose(current_marker_name_, rotated_cam_pose, markers_.front().marker.header);
  updateGUIValues(getMarkerByName(current_marker_name_));

  updateTrajectory();
}

void RvizCinematographerGUI::setMarkerFrames()
{
  for(auto& marker : markers_)
    marker.marker.header.frame_id = ui_.frame_line_edit->text().toStdString();

  updateServer(markers_);
  updateTrajectory();
}

void RvizCinematographerGUI::increaseMarkerScale()
{
  updateMarkerScales(1.1f);
}

void RvizCinematographerGUI::decreaseMarkerScale()
{
  updateMarkerScales(0.9f);
}

void RvizCinematographerGUI::showInteractiveMarkerControls()
{
  bool show_controls = ui_.show_interactive_marker_controls_check_box->isChecked();
  for(auto& marker : markers_)
    for(auto& control : marker.marker.controls)
      control.interaction_mode = show_controls ? visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE
                                               : visualization_msgs::msg::InteractiveMarkerControl::BUTTON;

  updateServer(markers_);
}

void RvizCinematographerGUI::updateMarkerScale(TimedMarker& marker,
                                               float scale_factor)
{
  marker.marker.scale *= scale_factor;
  marker.marker.controls[0].markers[0].scale.x *= scale_factor;
  marker.marker.controls[0].markers[0].scale.y *= scale_factor;
  marker.marker.controls[0].markers[0].scale.z *= scale_factor;
  marker.marker.controls[0].markers[1].scale.x *= scale_factor;
  marker.marker.controls[0].markers[1].scale.y *= scale_factor;
  marker.marker.controls[0].markers[1].scale.z *= scale_factor;
}

void RvizCinematographerGUI::updateMarkerScales(float scale_factor)
{
  for(auto& marker : markers_)
    updateMarkerScale(marker, scale_factor);

  updateServer(markers_);
}

std::string RvizCinematographerGUI::getTrajectoriesDirectory()
{
  try
  {
    return ament_index_cpp::get_package_share_directory("rviz2_cinematographer_gui") + "/trajectories/";
  }
  catch(const std::exception& e)
  {
    RCLCPP_WARN(getLogger(), "Could not find share directory of rviz2_cinematographer_gui: %s", e.what());
    return std::string();
  }
}

void RvizCinematographerGUI::loadTrajectoryFromFile()
{
  std::string directory_path = getTrajectoriesDirectory();

  QString file_name = QFileDialog::getOpenFileName(widget_, "Open Trajectory", QString(directory_path.c_str()),
                                                   "All Files (*);;.yaml files (*.yaml);;.txt files (*.txt)");
  if(file_name == "")
  {
    RCLCPP_ERROR_STREAM(getLogger(), "No file specified.");
    return;
  }

  std::string extension = std::filesystem::path(file_name.toStdString()).extension().string();

  bool loaded = false;
  if(extension == ".yaml")
  {
    loaded = loadTrajectoryFromYaml(file_name.toStdString());
  }
  else if(extension == ".txt")
  {
    loaded = loadTrajectoryFromTxt(file_name.toStdString());
  }
  else
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Specified file is neither .yaml nor .txt file.\n File name is: " << file_name.toStdString());
    return;
  }

  if(!loaded)
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Could not load any marker from file: " << file_name.toStdString());
    return;
  }

  // first marker is current marker
  markers_.front().marker.controls[0].markers[0].color.r = 0.f;
  markers_.front().marker.controls[0].markers[0].color.g = 1.f;
  current_marker_name_ = markers_.front().marker.name;

  server_->clear();
  updateServer(markers_);

  refillTable();

  updateGUIValues(markers_.front());
  updateTrajectory();
}

void RvizCinematographerGUI::saveTrajectoryToFile()
{
  std::string directory_path = getTrajectoriesDirectory();

  std::string file_path = QFileDialog::getSaveFileName(widget_, "Save Trajectory", QString(directory_path.c_str()),
                                                       "All Files (*)").toStdString();
  if(file_path.empty())
  {
    RCLCPP_ERROR_STREAM(getLogger(), "No file specified.");
  }
  else
  {
    std::string extension = std::filesystem::path(file_path).extension().string();

    if(extension != ".yaml")
      file_path = file_path + std::string(".yaml");

    safeTrajectoryToFile(file_path);
  }
}

void RvizCinematographerGUI::setVideoOutputPath()
{
  std::string directory_path = ui_.video_output_path_line_edit->text().toStdString();

  std::string file_path = QFileDialog::getSaveFileName(widget_, "Specify Path to Recorded Video",
                                                       QString(directory_path.c_str()),
                                                       ".avi files (*.avi)").toStdString();

  if(file_path.empty())
    return;

  std::filesystem::path path(file_path);
  if(path.extension().string() != ".avi")
    file_path = path.replace_extension(".avi").string();

  ui_.video_output_path_line_edit->setText(QString::fromStdString(file_path));
}

rviz2_cinematographer_msgs::msg::CameraMovement RvizCinematographerGUI::makeCameraMovement()
{
  rviz2_cinematographer_msgs::msg::CameraMovement cm;
  cm.eye.header.stamp = node_->now();
  cm.eye.header.frame_id = ui_.frame_line_edit->text().toStdString();
  cm.interpolation_speed = WAVE_INTERPOLATION_SPEED;
  cm.transition_duration = rclcpp::Duration::from_seconds(0.0);

  cm.up.header = cm.focus.header = cm.eye.header;

  cm.up.vector.x = 0.0;
  cm.up.vector.y = 0.0;
  cm.up.vector.z = 1.0;

  return cm;
}

visualization_msgs::msg::InteractiveMarker RvizCinematographerGUI::makeMarker(double x,
                                                                              double y,
                                                                              double z)
{
  visualization_msgs::msg::InteractiveMarker marker;
  marker.header.frame_id = ui_.frame_line_edit->text().toStdString();
  marker.name = "marker";
  marker.description = "Marker";
  marker.scale = 2.22f;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = M_SQRT1_2;
  marker.pose.orientation.y = M_SQRT1_2;

  makeBoxControl(marker);

  visualization_msgs::msg::InteractiveMarkerControl pose_control;
  pose_control.orientation.w = M_SQRT1_2;
  pose_control.orientation.x = M_SQRT1_2;
  pose_control.orientation.y = 0;
  pose_control.orientation.z = 0;
  pose_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  pose_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);

  pose_control.orientation.x = 0;
  pose_control.orientation.y = M_SQRT1_2;
  pose_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  pose_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);

  pose_control.orientation.y = 0;
  pose_control.orientation.z = M_SQRT1_2;
  pose_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
  marker.controls.push_back(pose_control);
  return marker;
}

void RvizCinematographerGUI::colorizeMarkersRed()
{
  for(auto& marker : markers_)
  {
    marker.marker.controls[0].markers[0].color.r = 1.f;
    marker.marker.controls[0].markers[0].color.g = 0.f;
  }
}

void RvizCinematographerGUI::appendMarkerToTrajectory(const MarkerIterator& goal_marker_iter,
                                                      rviz2_cinematographer_msgs::msg::CameraTrajectory::SharedPtr& cam_trajectory,
                                                      const MarkerIterator& last_marker_iter)
{
  rviz2_cinematographer_msgs::msg::CameraMovement cam_movement;
  convertMarkerToCamMovement(*goal_marker_iter, cam_movement);

  bool first_marker = cam_trajectory->trajectory.empty();
  bool accelerate = false;
  // accelerate if halted before
  if(!cam_trajectory->trajectory.empty())
    if(cam_trajectory->trajectory.back().interpolation_speed == DECLINING_INTERPOLATION_SPEED ||
       cam_trajectory->trajectory.back().interpolation_speed == WAVE_INTERPOLATION_SPEED)
      accelerate = true;

  cam_movement.interpolation_speed = (first_marker || accelerate) ? RISING_INTERPOLATION_SPEED
                                                                  : FULL_INTERPOLATION_SPEED;

  if(goal_marker_iter == last_marker_iter)
  {
    // if the whole trajectory is between the last two markers, use WAVE, else decline
    if(first_marker || cam_trajectory->trajectory.back().interpolation_speed == DECLINING_INTERPOLATION_SPEED ||
                       cam_trajectory->trajectory.back().interpolation_speed == WAVE_INTERPOLATION_SPEED)
      cam_movement.interpolation_speed = WAVE_INTERPOLATION_SPEED;
    else
      cam_movement.interpolation_speed = DECLINING_INTERPOLATION_SPEED;
  }

  // if camera should wait, add another static "movement" to the same pose
  if(goal_marker_iter->wait_duration > 0.01)
  {
    // adapt interpolation speed profile of previous movement to halt at marker we are waiting at
    if(cam_movement.interpolation_speed == RISING_INTERPOLATION_SPEED)
      cam_movement.interpolation_speed = WAVE_INTERPOLATION_SPEED;

    if(cam_movement.interpolation_speed == FULL_INTERPOLATION_SPEED)
      cam_movement.interpolation_speed = DECLINING_INTERPOLATION_SPEED;

    cam_trajectory->trajectory.push_back(cam_movement);

    cam_movement.transition_duration = rclcpp::Duration::from_seconds(goal_marker_iter->wait_duration);
    cam_movement.interpolation_speed = DECLINING_INTERPOLATION_SPEED;
    cam_trajectory->trajectory.push_back(cam_movement);
  }
  else
  {
    cam_trajectory->trajectory.push_back(cam_movement);
  }
}

void RvizCinematographerGUI::convertMarkerToCamMovement(const TimedMarker& marker,
                                                        rviz2_cinematographer_msgs::msg::CameraMovement& cam_movement)
{
  cam_movement = makeCameraMovement();
  cam_movement.transition_duration = rclcpp::Duration::from_seconds(marker.transition_duration);

  if(!ui_.use_up_of_world_check_box->isChecked())
  {
    // in the cam frame up is the negative x direction
    tf2::Vector3 rotated_vector = rotateVector(tf2::Vector3(-1, 0, 0), marker.marker.pose.orientation);
    cam_movement.up.vector.x = rotated_vector.x();
    cam_movement.up.vector.y = rotated_vector.y();
    cam_movement.up.vector.z = rotated_vector.z();
  }

  // look from
  cam_movement.eye.point = marker.marker.pose.position;

  // look at
  tf2::Vector3 rotated_vector = rotateVector(tf2::Vector3(0, 0, -1), marker.marker.pose.orientation);
  cam_movement.focus.point.x = marker.marker.pose.position.x + ui_.smoothness_spin_box->value() * rotated_vector.x();
  cam_movement.focus.point.y = marker.marker.pose.position.y + ui_.smoothness_spin_box->value() * rotated_vector.y();
  cam_movement.focus.point.z = marker.marker.pose.position.z + ui_.smoothness_spin_box->value() * rotated_vector.z();
}

void RvizCinematographerGUI::publishRecordParams()
{
  rviz2_cinematographer_msgs::msg::Record record_params;
  record_params.do_record = true;
  record_params.path_to_output = ui_.video_output_path_line_edit->text().toStdString();
  record_params.frames_per_second = static_cast<int8_t>(ui_.video_fps_spin_box->value());
  record_params.compress = ui_.video_compressed_check_box->isChecked();
  record_params.add_watermark = ui_.watermark_check_box->isChecked();
  record_params_pub_->publish(record_params);
}

void
RvizCinematographerGUI::recordFinishedCallback(const rviz2_cinematographer_msgs::msg::Finished::ConstSharedPtr record_finished)
{
  if(record_finished->is_finished)
    postToGuiThread([this]() { ui_.record_radio_button->setChecked(false); });
}

void RvizCinematographerGUI::moveCamToCurrent()
{
  if(ui_.record_radio_button->isChecked())
    publishRecordParams();

  moveCamToMarker(current_marker_name_, 0.5);
}

void RvizCinematographerGUI::moveCamToFirst()
{
  // do nothing if current is already the first marker
  if(markers_.begin()->marker.name == current_marker_name_)
    return;

  if(ui_.record_radio_button->isChecked())
    publishRecordParams();

  auto it = findMarker(current_marker_name_);
  if(it == markers_.end())
    return;

  // fill Camera Trajectory msg with markers and times
  auto cam_trajectory = std::make_shared<rviz2_cinematographer_msgs::msg::CameraTrajectory>();
  cam_trajectory->target_frame = ui_.frame_line_edit->text().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_check_box->isChecked();

  if(ui_.splines_check_box->isChecked())
  {
    MarkerList markers;
    auto current = it;
    // used spline type uses first an last point but doesn't interpolate between them
    // therefore we add the marker before the current - or itself if there is none before
    if(current == std::prev(markers_.end()))
      markers.push_back(*current);
    else
      markers.push_back(*std::next(current));

    // then we add all other markers
    do
    {
      markers.push_back(*current);
    }
    while(current-- != markers_.begin());
    // and the last one a second time
    markers.push_back(*(markers_.begin()));

    markersToSplinedCamTrajectory(markers, cam_trajectory);
  }
  else
  {
    auto previous = it;
    do
    {
      previous--;
      appendMarkerToTrajectory(previous, cam_trajectory, markers_.begin());
    }
    while(previous != markers_.begin());
  }

  setCurrentFromTo(*it, *(markers_.begin()));

  // publish cam trajectory
  camera_trajectory_pub_->publish(*cam_trajectory);

  ui_.marker_table_widget->selectRow(getMarkerId(current_marker_name_));
}

void RvizCinematographerGUI::moveCamToPrev()
{
  // do nothing if current is already the first marker
  if(markers_.begin()->marker.name == current_marker_name_)
    return;

  if(ui_.record_radio_button->isChecked())
    publishRecordParams();

  auto it = findMarker(current_marker_name_);
  if(it == markers_.end())
    return;

  setCurrentFromTo(*it, *std::prev(it));

  moveCamToMarker(current_marker_name_);
}

void RvizCinematographerGUI::moveCamToNext()
{
  // do nothing if current is already the last marker
  if(std::prev(markers_.end())->marker.name == current_marker_name_)
    return;

  if(ui_.record_radio_button->isChecked())
    publishRecordParams();

  auto it = findMarker(current_marker_name_);
  if(it == markers_.end())
    return;

  setCurrentFromTo(*it, *std::next(it));

  moveCamToMarker(current_marker_name_);
}

void RvizCinematographerGUI::moveCamToLast()
{
  // do nothing if current is already the last marker
  if(std::prev(markers_.end())->marker.name == current_marker_name_)
    return;

  if(ui_.record_radio_button->isChecked())
    publishRecordParams();

  auto it = findMarker(current_marker_name_);
  if(it == markers_.end())
    return;

  // fill Camera Trajectory msg with markers and times
  auto cam_trajectory = std::make_shared<rviz2_cinematographer_msgs::msg::CameraTrajectory>();
  cam_trajectory->target_frame = ui_.frame_line_edit->text().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_check_box->isChecked();

  if(ui_.splines_check_box->isChecked())
  {
    MarkerList markers;
    auto current = it;
    // used spline type uses first an last point but doesn't interpolate between them
    // therefore we add the marker before the current - or itself if there is none before
    if(current == markers_.begin())
      markers.push_back(*current);
    else
      markers.push_back(*std::prev(current));

    // then we add all other markers
    for(; current != markers_.end(); current++)
      markers.push_back(*current);

    // and the last one a second time
    markers.push_back(*(std::prev(markers_.end())));

    markersToSplinedCamTrajectory(markers, cam_trajectory);
  }
  else
  {
    auto next = it;
    for(++next; next != markers_.end(); next++)
    {
      appendMarkerToTrajectory(next, cam_trajectory, std::prev(markers_.end()));
    }
  }

  setCurrentFromTo(*it, *(std::prev(markers_.end())));

  // publish cam trajectory
  camera_trajectory_pub_->publish(*cam_trajectory);

  ui_.marker_table_widget->selectRow(getMarkerId(current_marker_name_));
}

void RvizCinematographerGUI::moveCamToMarker(const std::string& marker_name,
                                             double transition_duration)
{
  const TimedMarker& marker = getMarkerByName(marker_name);

  auto cam_trajectory = std::make_shared<rviz2_cinematographer_msgs::msg::CameraTrajectory>();
  cam_trajectory->target_frame = ui_.frame_line_edit->text().toStdString();
  cam_trajectory->allow_free_yaw_axis = !ui_.use_up_of_world_check_box->isChecked();

  rviz2_cinematographer_msgs::msg::CameraMovement cam_movement;
  convertMarkerToCamMovement(marker, cam_movement);
  if(transition_duration >= 0.0)
    cam_movement.transition_duration = rclcpp::Duration::from_seconds(transition_duration);
  cam_movement.interpolation_speed = WAVE_INTERPOLATION_SPEED;

  cam_trajectory->trajectory.push_back(cam_movement);

  if(marker.wait_duration > 0.01)
  {
    cam_movement.transition_duration = rclcpp::Duration::from_seconds(marker.wait_duration);
    cam_trajectory->trajectory.push_back(cam_movement);
  }

  camera_trajectory_pub_->publish(*cam_trajectory);

  ui_.marker_table_widget->selectRow(getMarkerId(current_marker_name_));
}

void RvizCinematographerGUI::updateGUIValues(const TimedMarker& current_marker)
{
  setValueQuietly(ui_.translation_x_spin_box, current_marker.marker.pose.position.x);
  setValueQuietly(ui_.translation_y_spin_box, current_marker.marker.pose.position.y);
  setValueQuietly(ui_.translation_z_spin_box, current_marker.marker.pose.position.z);

  setValueQuietly(ui_.rotation_x_spin_box, current_marker.marker.pose.orientation.x);
  setValueQuietly(ui_.rotation_y_spin_box, current_marker.marker.pose.orientation.y);
  setValueQuietly(ui_.rotation_z_spin_box, current_marker.marker.pose.orientation.z);
  setValueQuietly(ui_.rotation_w_spin_box, current_marker.marker.pose.orientation.w);

  if(current_marker.marker.name.empty())
    return;

  int marker_index = getMarkerId(current_marker.marker.name);

  auto transition_duration_spin_box = qobject_cast<QDoubleSpinBox*>(ui_.marker_table_widget->cellWidget(marker_index, 0));
  if(transition_duration_spin_box)
    setValueQuietly(transition_duration_spin_box, current_marker.transition_duration);

  auto wait_duration_spin_box = qobject_cast<QDoubleSpinBox*>(ui_.marker_table_widget->cellWidget(marker_index, 1));
  if(wait_duration_spin_box)
    setValueQuietly(wait_duration_spin_box, current_marker.wait_duration);
}

void RvizCinematographerGUI::setValueQuietly(QDoubleSpinBox* spin_box,
                                             double value)
{
  bool old_block_state = spin_box->blockSignals(true);
  spin_box->setValue(value);
  spin_box->blockSignals(old_block_state);
}

void RvizCinematographerGUI::processFeedback(const FeedbackConstPtr& feedback)
{
  // update markers
  visualization_msgs::msg::InteractiveMarker marker;
  if(feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP
     && server_->get(feedback->marker_name, marker))
  {
    current_marker_name_ = feedback->marker_name;

    updateGUIValues(getMarkerByName(feedback->marker_name));
    ui_.marker_table_widget->selectRow(getMarkerId(current_marker_name_));

    // update marker pose
    marker.pose = feedback->pose;
    getMarkerByName(feedback->marker_name).marker.pose = feedback->pose;

    colorizeMarkersRed();
    // change color of current marker to green
    getMarkerByName(feedback->marker_name).marker.controls[0].markers[0].color.r = 0.f;
    getMarkerByName(feedback->marker_name).marker.controls[0].markers[0].color.g = 1.f;

    server_->clear();
    updateServer(markers_);

    updateTrajectory();
  }
}

void RvizCinematographerGUI::updateCurrentMarker()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = ui_.translation_x_spin_box->value();
  pose.position.y = ui_.translation_y_spin_box->value();
  pose.position.z = ui_.translation_z_spin_box->value();

  pose.orientation.x = ui_.rotation_x_spin_box->value();
  pose.orientation.y = ui_.rotation_y_spin_box->value();
  pose.orientation.z = ui_.rotation_z_spin_box->value();
  pose.orientation.w = ui_.rotation_w_spin_box->value();

  TimedMarker& current_marker = getMarkerByName(current_marker_name_);
  current_marker.marker.pose = pose;

  int marker_index = getMarkerId(current_marker.marker.name);

  auto transition_duration_spin_box = qobject_cast<QDoubleSpinBox*>(ui_.marker_table_widget->cellWidget(marker_index, 0));
  if(transition_duration_spin_box)
    current_marker.transition_duration = transition_duration_spin_box->value();

  auto wait_duration_spin_box = qobject_cast<QDoubleSpinBox*>(ui_.marker_table_widget->cellWidget(marker_index, 1));
  if(wait_duration_spin_box)
    current_marker.wait_duration = wait_duration_spin_box->value();

  server_->setPose(current_marker_name_, pose, markers_.front().marker.header);
  server_->applyChanges();

  updateTrajectory();
}

void RvizCinematographerGUI::updateMarker()
{
  auto duration_spin_box = qobject_cast<QDoubleSpinBox*>(sender());
  if(duration_spin_box)
  {
    int row = duration_spin_box->property("row").toInt();
    int col = duration_spin_box->property("column").toInt();

    std::string marker_name = std::to_string(row + 1);
    current_marker_name_ = marker_name;

    TimedMarker& current_marker = getMarkerByName(marker_name);
    if(col == 0)
      current_marker.transition_duration = duration_spin_box->value();
    else
      current_marker.wait_duration = duration_spin_box->value();

    colorizeMarkersRed();
    // change color of current marker to green
    getMarkerByName(marker_name).marker.controls[0].markers[0].color.r = 0.f;
    getMarkerByName(marker_name).marker.controls[0].markers[0].color.g = 1.f;

    server_->clear();
    updateServer(markers_);

    ui_.marker_table_widget->selectRow(row);
  }
}

void RvizCinematographerGUI::updateWhoIsCurrentMarker(int marker_id)
{
  std::string marker_name = std::to_string(marker_id + 1);
  current_marker_name_ = marker_name;

  colorizeMarkersRed();
  // change color of current marker to green
  getMarkerByName(marker_name).marker.controls[0].markers[0].color.r = 0.f;
  getMarkerByName(marker_name).marker.controls[0].markers[0].color.g = 1.f;

  server_->clear();
  updateServer(markers_);

  updateGUIValues(getMarkerByName(current_marker_name_));
}

tf2::Vector3 RvizCinematographerGUI::rotateVector(const tf2::Vector3& vector,
                                                  const geometry_msgs::msg::Quaternion& quat)
{
  return tf2::quatRotate(quaternionFromMsg(quat), vector);
}

void RvizCinematographerGUI::markersToSplinedCamTrajectory(const MarkerList& markers,
                                                           rviz2_cinematographer_msgs::msg::CameraTrajectory::SharedPtr trajectory)
{
  std::vector<Vector3> input_eye_positions;
  std::vector<Vector3> input_focus_positions;
  std::vector<Vector3> input_up_directions;
  prepareSpline(markers, input_eye_positions, input_focus_positions, input_up_directions);

  // Generate splines
  UniformCRSpline<Vector3> eye_spline(input_eye_positions);
  UniformCRSpline<Vector3> focus_spline(input_focus_positions);
  UniformCRSpline<Vector3> up_spline(input_up_directions);

  std::vector<double> transition_durations;
  std::vector<double> wait_durations;
  double total_transition_duration = 0.0;
  computeDurations(markers, transition_durations, wait_durations, total_transition_duration);

  splineToCamTrajectory(eye_spline,
                        focus_spline,
                        up_spline,
                        transition_durations,
                        wait_durations,
                        total_transition_duration,
                        trajectory);
}

void RvizCinematographerGUI::prepareSpline(const MarkerList& markers,
                                           std::vector<Vector3>& input_eye_positions,
                                           std::vector<Vector3>& input_focus_positions,
                                           std::vector<Vector3>& input_up_directions)
{
  Vector3 position;
  for(const auto& marker : markers)
  {
    position[0] = static_cast<float>(marker.marker.pose.position.x);
    position[1] = static_cast<float>(marker.marker.pose.position.y);
    position[2] = static_cast<float>(marker.marker.pose.position.z);
    input_eye_positions.push_back(position);

    tf2::Vector3 rotated_vector = rotateVector(tf2::Vector3(0, 0, -ui_.smoothness_spin_box->value()),
                                               marker.marker.pose.orientation);
    position[0] = position[0] + static_cast<float>(rotated_vector.x());
    position[1] = position[1] + static_cast<float>(rotated_vector.y());
    position[2] = position[2] + static_cast<float>(rotated_vector.z());
    input_focus_positions.push_back(position);

    if(!ui_.use_up_of_world_check_box->isChecked())
    {
      // in the cam frame up is the negative x direction
      tf2::Vector3 rotated_up_vector = rotateVector(tf2::Vector3(-1, 0, 0), marker.marker.pose.orientation);
      position[0] = static_cast<float>(rotated_up_vector.x());
      position[1] = static_cast<float>(rotated_up_vector.y());
      position[2] = static_cast<float>(rotated_up_vector.z());
    }
    else
    {
      position[0] = 0;
      position[1] = 0;
      position[2] = 1;
    }
    input_up_directions.push_back(position);
  }
}

void RvizCinematographerGUI::computeDurations(const MarkerList& markers,
                                              std::vector<double>& transition_durations,
                                              std::vector<double>& wait_durations,
                                              double& total_transition_duration)
{
  const double frequency = ui_.publish_rate_spin_box->value();
  const bool smooth_velocity = ui_.smooth_velocity_check_box->isChecked();

  int counter = 0;
  for(const auto& marker : markers)
  {
    // skip first because this marker is only used for the spline
    // skip second because we don't use the timing of the start marker
    if(counter > 1)
    {
      if(smooth_velocity)
        total_transition_duration += marker.transition_duration;
      else
      {
        transition_durations.push_back(marker.transition_duration / frequency);
        wait_durations.push_back(marker.wait_duration);
      }
    }

    counter++;
  }
}

void RvizCinematographerGUI::splineToCamTrajectory(const UniformCRSpline<Vector3>& eye_spline,
                                                   const UniformCRSpline<Vector3>& focus_spline,
                                                   const UniformCRSpline<Vector3>& up_spline,
                                                   const std::vector<double>& transition_durations,
                                                   const std::vector<double>& wait_durations,
                                                   const double total_transition_duration,
                                                   rviz2_cinematographer_msgs::msg::CameraTrajectory::SharedPtr trajectory)
{
  const double frequency = ui_.publish_rate_spin_box->value();
  const bool smooth_velocity = ui_.smooth_velocity_check_box->isChecked();

  // rate to sample from spline and get points
  rviz2_cinematographer_msgs::msg::CameraMovement cam_movement = makeCameraMovement();
  double rate = 1.0 / frequency;
  double max_t = eye_spline.getMaxT();
  double total_length = eye_spline.totalLength();
  bool first = true;
  bool last_run = false;
  int current_transition_id = 0;
  int previous_transition_id = 0;
  for(double t = 0.0; t <= max_t;)
  {
    // get position in spline
    auto interpolated_position = eye_spline.getPosition(static_cast<float>(t));
    auto interpolated_focus = focus_spline.getPosition(static_cast<float>(t));
    auto interpolated_up = up_spline.getPosition(static_cast<float>(t));

    cam_movement.eye.point.x = interpolated_position[0];
    cam_movement.eye.point.y = interpolated_position[1];
    cam_movement.eye.point.z = interpolated_position[2];
    cam_movement.focus.point.x = interpolated_focus[0];
    cam_movement.focus.point.y = interpolated_focus[1];
    cam_movement.focus.point.z = interpolated_focus[2];

    if(!ui_.use_up_of_world_check_box->isChecked())
    {
      cam_movement.up.vector.x = interpolated_up[0];
      cam_movement.up.vector.y = interpolated_up[1];
      cam_movement.up.vector.z = interpolated_up[2];
    }
    // else is not necessary - up is already set to default in makeCameraMovement

    bool accelerate = false;
    if(!trajectory->trajectory.empty())
      accelerate = trajectory->trajectory.back().interpolation_speed == DECLINING_INTERPOLATION_SPEED;

    cam_movement.interpolation_speed = (first || accelerate) ? RISING_INTERPOLATION_SPEED
                                                             : FULL_INTERPOLATION_SPEED;

    // decline at end of trajectory and when reaching next position and velocity is not smoothed
    if((!smooth_velocity && current_transition_id != previous_transition_id) || last_run)
      cam_movement.interpolation_speed = DECLINING_INTERPOLATION_SPEED;

    double transition_duration = 0.0;
    if(smooth_velocity)
    {
      double local_length = eye_spline.arcLength(static_cast<float>(std::max(t - rate, 0.0)), static_cast<float>(t));
      if(total_length > 0.0)
        transition_duration = total_transition_duration * local_length / total_length;
    }
    else
    {
      size_t index = static_cast<size_t>(std::floor(std::max(t - rate, 0.0)));
      if(index < transition_durations.size())
        transition_duration = transition_durations[index];
    }

    cam_movement.transition_duration = rclcpp::Duration::from_seconds(transition_duration);


    // recreate movement/marker id to wait after transition if waiting time specified
    current_transition_id = (int)std::floor(t + 0.00001); // magic number needed due to arithmetic imprecision with doubles
    if(!smooth_velocity && current_transition_id != previous_transition_id &&
       static_cast<size_t>(previous_transition_id) < wait_durations.size() &&
       wait_durations[previous_transition_id] > 0.01)
    {
      cam_movement.interpolation_speed = DECLINING_INTERPOLATION_SPEED;
      trajectory->trajectory.push_back(cam_movement);

      cam_movement.transition_duration = rclcpp::Duration::from_seconds(wait_durations[previous_transition_id]);
      trajectory->trajectory.push_back(cam_movement);
    }
    else
    {
      trajectory->trajectory.push_back(cam_movement);
    }
    previous_transition_id = current_transition_id;

    RCLCPP_DEBUG_STREAM(getLogger(), "t " << t << " max_t " << max_t);

    if(last_run)
      break;

    t += rate;
    if(t > max_t)
    {
      last_run = true;
      t = max_t;
    }

    first = false;
  }
}

void RvizCinematographerGUI::markersToSplinedPoses(const MarkerList& markers,
                                                   std::vector<geometry_msgs::msg::Pose>& spline_poses,
                                                   double frequency,
                                                   bool duplicate_ends)
{
  // put all markers positions into a vector. First and last double.
  std::vector<Vector3> spline_points;
  Vector3 position;
  bool first = true;
  for(const auto& marker : markers)
  {
    position[0] = static_cast<float>(marker.marker.pose.position.x);
    position[1] = static_cast<float>(marker.marker.pose.position.y);
    position[2] = static_cast<float>(marker.marker.pose.position.z);
    spline_points.push_back(position);

    if(first && duplicate_ends)
      spline_points.push_back(position);

    first = false;
  }
  if(duplicate_ends)
    spline_points.push_back(position);

  UniformCRSpline<Vector3> spline(spline_points);

  // rate to sample from spline and get points
  double rate = 1.0 / frequency;
  float max_t = spline.getMaxT();
  auto current_marker = markers.begin();
  auto next_marker = std::next(markers.begin());
  int current_marker_id = 0;
  bool last_run = false;
  for(double i = 0.f; i <= max_t;)
  {
    // get position of spline
    auto interpolated_position = spline.getPosition(static_cast<float>(i));
    geometry_msgs::msg::Pose pose;
    pose.position.x = interpolated_position[0];
    pose.position.y = interpolated_position[1];
    pose.position.z = interpolated_position[2];

    // i from 0 to 1 corresponds to the spline between the first and the second marker
    // we have to maintain iterators for slerp
    if(current_marker_id != (int)std::floor(i) && !last_run && std::next(next_marker) != markers.end())
    {
      current_marker_id++;
      current_marker++;
      next_marker++;
    }

    // get slerped orientation
    double slerp_factor = fmod(i, 1.0);
    if(last_run)
      slerp_factor = 1.0;
    pose.orientation = slerp(current_marker->marker.pose.orientation, next_marker->marker.pose.orientation, slerp_factor);

    spline_poses.push_back(pose);

    if(last_run)
      break;

    i += rate;
    if(i >= max_t)
    {
      last_run = true;
      i = max_t;
    }
  }
}

} // namespace
