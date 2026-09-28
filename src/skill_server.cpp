#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

namespace
{
geometry_msgs::msg::Pose pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose result;
  result.position.x = x;
  result.position.y = y;
  result.position.z = z;
  // Tool Z points downward: 180 degrees about Y. This wrist branch avoids the
  // UR3e shoulder-limit solution selected by the equivalent X-axis rotation.
  result.orientation.y = 1.0;
  result.orientation.w = 0.0;
  return result;
}

geometry_msgs::msg::Pose upright_pose(const geometry_msgs::msg::Pose & input)
{
  auto result = input;
  result.orientation.x = 0.0;
  result.orientation.y = 0.0;
  result.orientation.z = 0.0;
  result.orientation.w = 1.0;
  return result;
}
}  // namespace

class SkillServer : public rclcpp::Node
{
public:
  using ExecuteSkill = rcl_interfaces::srv::SetParametersAtomically;
  using GripperCommand = control_msgs::action::ParallelGripperCommand;

  SkillServer()
  : Node("skill_server"),
    initial_object_poses_{{"red_cube", pose(0.25, -0.20, 0.085)},
      {"yellow_cube", pose(0.25, 0.0, 0.085)}, {"blue_cube", pose(0.25, 0.20, 0.085)}},
    object_poses_(initial_object_poses_),
    zone_poses_{{"zone_a", pose(0.38, -0.12, 0.085)},
      {"zone_b", pose(0.38, 0.0, 0.085)}, {"zone_c", pose(0.38, 0.12, 0.085)}}
  {
    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_ = create_service<ExecuteSkill>(
      "/execute_skill",
      [this](const ExecuteSkill::Request::SharedPtr request,
        ExecuteSkill::Response::SharedPtr response) {execute(request, response);},
      rclcpp::ServicesQoS(), group_);
    gz_pose_client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(
      "/world/hri_assignment/set_pose");
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "/robotiq_gripper_controller/gripper_cmd");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    follow_timer_ = create_wall_timer(50ms, [this]() {follow_held_object();});
    scene_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/hri_scene_markers", rclcpp::QoS(1).reliable().transient_local());
    publish_zone_markers();
    // MoveGroupInterface needs shared_from_this(), so initialize it from a
    // timer after construction. Retrying also handles move_group starting a
    // little later than this node.
    startup_timer_ = create_wall_timer(1s, [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      try {
        initialize();
        if (world_added_) {
          startup_timer_->cancel();
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "Waiting to initialize MoveIt scene: %s", error.what());
      }
    });
    RCLCPP_INFO(
      get_logger(), "Skill server ready: group=ur_manipulator, frame=base_link, eef=gripper_tcp");
  }

private:
  void initialize()
  {
    if (!move_group_) {
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ur_manipulator");
      move_group_->setEndEffectorLink("gripper_tcp");
      move_group_->setPoseReferenceFrame("base_link");
      // OMPL finds collision-free paths around the tabletop. Pilz PTP uses a
      // joint-space interpolation that cuts through the table for this compact
      // UR3e workspace even when both endpoint poses are valid.
      move_group_->setPlanningPipelineId("ompl");
      move_group_->setPlannerId("RRTConnectkConfigDefault");
      move_group_->setMaxVelocityScalingFactor(0.08);
      move_group_->setMaxAccelerationScalingFactor(0.08);
      move_group_->setPlanningTime(10.0);
      move_group_->setNumPlanningAttempts(10);
    }
    if (!world_added_) {
      world_added_ = add_world();
    }
  }

  static moveit_msgs::msg::CollisionObject box(
    const std::string & id, const geometry_msgs::msg::Pose & object_pose,
    const std::vector<double> & dimensions)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "base_link";
    object.id = id;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.assign(dimensions.begin(), dimensions.end());
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(object_pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
  }

  static std_msgs::msg::ColorRGBA color(
    double red, double green, double blue, double alpha = 1.0)
  {
    std_msgs::msg::ColorRGBA result;
    result.r = red;
    result.g = green;
    result.b = blue;
    result.a = alpha;
    return result;
  }

  static std_msgs::msg::ColorRGBA object_color(const std::string & name)
  {
    if (name == "red_cube") {
      return color(0.95, 0.03, 0.03);
    }
    if (name == "yellow_cube") {
      return color(1.0, 0.78, 0.03);
    }
    if (name == "blue_cube") {
      return color(0.03, 0.12, 0.95);
    }
    return color(0.52, 0.32, 0.16);
  }

  bool add_world()
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    std::vector<moveit_msgs::msg::ObjectColor> colors;
    auto table_pose = upright_pose(pose(0.47, 0.0, 0.03));
    objects.push_back(box("work_table", table_pose, {0.56, 0.65, 0.06}));
    moveit_msgs::msg::ObjectColor table_color;
    table_color.id = "work_table";
    table_color.color = object_color("work_table");
    colors.push_back(table_color);
    for (const auto & [name, object_pose] : object_poses_) {
      auto upright = upright_pose(object_pose);
      objects.push_back(box(name, upright, {0.05, 0.05, 0.05}));
      moveit_msgs::msg::ObjectColor cube_color;
      cube_color.id = name;
      cube_color.color = object_color(name);
      colors.push_back(cube_color);
    }
    const bool applied = planning_scene_.applyCollisionObjects(objects, colors);
    if (applied) {
      RCLCPP_INFO(get_logger(), "Added table and three colored cubes to the MoveIt PlanningScene");
    } else {
      RCLCPP_WARN(get_logger(), "MoveIt PlanningScene is not ready; scene update will be retried");
    }
    return applied;
  }

  void add_marker_box(
    visualization_msgs::msg::MarkerArray & array, int & id,
    const std::string & name, double x, double y, double z,
    double size_x, double size_y, double size_z,
    const std_msgs::msg::ColorRGBA & marker_color)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = now();
    marker.ns = name;
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = size_x;
    marker.scale.y = size_y;
    marker.scale.z = size_z;
    marker.color = marker_color;
    array.markers.push_back(marker);
  }

  void publish_zone_markers()
  {
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    const std::vector<std::tuple<std::string, double, double, std_msgs::msg::ColorRGBA>> zones = {
      {"A", 0.38, -0.12, color(0.9, 0.05, 0.05, 0.85)},
      {"B", 0.38, 0.0, color(1.0, 0.78, 0.03, 0.85)},
      {"C", 0.38, 0.12, color(0.05, 0.2, 1.0, 0.85)}};
    for (const auto & [label, x, y, zone_color] : zones) {
      const auto marker_namespace = "zone_" + label;
      add_marker_box(array, id, marker_namespace, x, y, 0.062, 0.09, 0.09, 0.004, zone_color);
      add_marker_box(array, id, marker_namespace, x + 0.042, y, 0.071, 0.006, 0.09, 0.018, zone_color);
      add_marker_box(array, id, marker_namespace, x - 0.042, y, 0.071, 0.006, 0.09, 0.018, zone_color);
      add_marker_box(array, id, marker_namespace, x, y + 0.042, 0.071, 0.078, 0.006, 0.018, zone_color);
      add_marker_box(array, id, marker_namespace, x, y - 0.042, 0.071, 0.078, 0.006, 0.018, zone_color);

      visualization_msgs::msg::Marker text;
      text.header.frame_id = "base_link";
      text.header.stamp = now();
      text.ns = marker_namespace;
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = x + 0.075;
      text.pose.position.y = y;
      text.pose.position.z = 0.105;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.035;
      text.color = color(1.0, 1.0, 1.0);
      text.text = label;
      array.markers.push_back(text);
    }
    scene_marker_publisher_->publish(array);
  }

  void log_joint_state(
    const std::string & prefix,
    const trajectory_msgs::msg::JointTrajectory & trajectory)
  {
    if (trajectory.points.empty()) {
      return;
    }
    const auto & positions = trajectory.points.back().positions;
    std::string values;
    for (std::size_t index = 0; index < positions.size(); ++index) {
      if (index > 0) {
        values += ", ";
      }
      const auto & name = index < trajectory.joint_names.size() ?
        trajectory.joint_names[index] : std::to_string(index);
      values += name + "=" + std::to_string(positions[index]);
    }
    RCLCPP_DEBUG(get_logger(), "%s: %s", prefix.c_str(), values.c_str());
  }

  void log_measured_joints(const std::string & prefix)
  {
    const auto names = move_group_->getJointNames();
    const auto positions = move_group_->getCurrentJointValues();
    std::string values;
    for (std::size_t index = 0; index < positions.size(); ++index) {
      if (index > 0) {
        values += ", ";
      }
      const auto & name = index < names.size() ? names[index] : std::to_string(index);
      values += name + "=" + std::to_string(positions[index]);
    }
    RCLCPP_ERROR(get_logger(), "%s: %s", prefix.c_str(), values.c_str());
  }

  bool set_bounded_start(std::string & reason)
  {
    auto current = move_group_->getCurrentState(2.0);
    if (!current) {
      reason = "ROBOT_STATE_UNAVAILABLE";
      return false;
    }
    // A trajectory may finish a few encoder ticks outside a revolute bound.
    // Normalize continuous joints and clamp only numerical boundary drift
    // before handing the measured state back to MoveIt.
    current->enforceBounds();
    move_group_->setStartState(*current);
    return true;
  }

  void unwrap_continuous_joint(
    trajectory_msgs::msg::JointTrajectory & trajectory, const std::string & joint_name)
  {
    const auto planned = std::find(
      trajectory.joint_names.begin(), trajectory.joint_names.end(), joint_name);
    const auto measured_names = move_group_->getJointNames();
    const auto measured = move_group_->getCurrentJointValues();
    const auto actual = std::find(measured_names.begin(), measured_names.end(), joint_name);
    if (planned == trajectory.joint_names.end() || actual == measured_names.end()) {
      return;
    }
    const auto planned_index = std::distance(trajectory.joint_names.begin(), planned);
    const auto actual_index = std::distance(measured_names.begin(), actual);
    if (actual_index >= static_cast<long>(measured.size())) {
      return;
    }
    constexpr double two_pi = 6.283185307179586;
    double reference = measured[actual_index];
    for (auto & point : trajectory.points) {
      if (planned_index >= static_cast<long>(point.positions.size())) {
        continue;
      }
      auto & position = point.positions[planned_index];
      position += std::round((reference - position) / two_pi) * two_pi;
      reference = position;
    }
  }

  bool move_to(
    const geometry_msgs::msg::Pose & target, std::string & reason,
    bool keep_current_ik_branch = false)
  {
    RCLCPP_INFO(
      get_logger(), "Motion target gripper_tcp: x=%.3f y=%.3f z=%.3f",
      target.position.x, target.position.y, target.position.z);
    if (!set_bounded_start(reason)) {
      return false;
    }
    if (keep_current_ik_branch) {
      // Resolve vertical grasp/lift segments to the IK solution nearest the
      // measured state so the arm does not select an equivalent multi-turn path.
      if (!move_group_->setJointValueTarget(target, "gripper_tcp")) {
        reason = "IK_FAILED";
        return false;
      }
    } else {
      // For long transfers let OMPL sample other IK branches; the nearest
      // solution can fold the gripper back into the upper arm.
      move_group_->setPoseTarget(target, "gripper_tcp");
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 0; attempt < 3 && !planned; ++attempt) {
      planned = static_cast<bool>(move_group_->plan(plan));
    }
    if (!planned) {
      reason = "PLANNING_FAILED";
      move_group_->clearPoseTargets();
      return false;
    }
    unwrap_continuous_joint(plan.trajectory.joint_trajectory, "wrist_3_joint");
    log_joint_state("Planned joint goal", plan.trajectory.joint_trajectory);
    bool executed = static_cast<bool>(move_group_->execute(plan));
    if (!executed) {
      log_measured_joints("Measured joints after controller abort");
      // Replan once from the measured stopped state after a controller abort.
      if (!set_bounded_start(reason)) {
        move_group_->clearPoseTargets();
        return false;
      }
      if (keep_current_ik_branch) {
        if (!move_group_->setJointValueTarget(target, "gripper_tcp")) {
          reason = "IK_FAILED";
          return false;
        }
      } else {
        move_group_->setPoseTarget(target, "gripper_tcp");
      }
      moveit::planning_interface::MoveGroupInterface::Plan retry;
      bool replanned = false;
      for (int attempt = 0; attempt < 3 && !replanned; ++attempt) {
        replanned = static_cast<bool>(move_group_->plan(retry));
      }
      if (replanned) {
        unwrap_continuous_joint(retry.trajectory.joint_trajectory, "wrist_3_joint");
        log_joint_state("Retry joint goal", retry.trajectory.joint_trajectory);
      }
      executed = replanned && static_cast<bool>(move_group_->execute(retry));
      if (!executed) {
        log_measured_joints("Measured joints after retry abort");
      }
    }
    move_group_->clearPoseTargets();
    if (!executed) {
      reason = "EXECUTION_FAILED";
    }
    return executed;
  }

  bool move_to_joint_values(const std::vector<double> & target, std::string & reason)
  {
    if (!set_bounded_start(reason) || !move_group_->setJointValueTarget(target)) {
      reason = "JOINT_TARGET_FAILED";
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group_->plan(plan))) {
      reason = "PLANNING_FAILED";
      return false;
    }
    unwrap_continuous_joint(plan.trajectory.joint_trajectory, "wrist_3_joint");
    if (!static_cast<bool>(move_group_->execute(plan))) {
      reason = "EXECUTION_FAILED";
      return false;
    }
    return true;
  }

  bool home(std::string & reason)
  {
    if (!set_bounded_start(reason)) {
      return false;
    }
    // The local hri_home state matches the collision-free startup pose. The
    // vendor "up" state was designed for a bare wrist; with a long 2F-85 its
    // transition can sweep a finger through upper_arm_link.
    if (!move_group_->setNamedTarget("hri_home")) {
      reason = "INVALID_HOME_TARGET";
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 0; attempt < 5 && !planned; ++attempt) {
      planned = static_cast<bool>(move_group_->plan(plan));
    }
    if (!planned) {
      reason = "PLANNING_FAILED";
      return false;
    }
    unwrap_continuous_joint(plan.trajectory.joint_trajectory, "wrist_3_joint");
    if (!static_cast<bool>(move_group_->execute(plan))) {
      reason = "EXECUTION_FAILED";
      return false;
    }
    return true;
  }

  void set_gazebo_pose(const std::string & name, const geometry_msgs::msg::Pose & target)
  {
    if (!gz_pose_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Gazebo set-pose bridge unavailable; PlanningScene remains authoritative");
      return;
    }
    auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = name;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    request->pose = target;
    gz_pose_client_->async_send_request(request);
  }

  void follow_held_object()
  {
    std::string object;
    {
      std::lock_guard<std::mutex> lock(visual_mutex_);
      object = visual_held_;
    }
    if (object.empty() || !tf_buffer_) {
      return;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        "base_link", "gripper_tcp", tf2::TimePointZero);
      geometry_msgs::msg::Pose cube_pose;
      cube_pose.position.x = transform.transform.translation.x;
      cube_pose.position.y = transform.transform.translation.y;
      cube_pose.position.z = transform.transform.translation.z;
      cube_pose.orientation = transform.transform.rotation;
      set_gazebo_pose(object, cube_pose);
    } catch (const tf2::TransformException &) {
      // TF is briefly unavailable during startup; the next timer tick retries.
    }
  }

  void set_visual_held(const std::string & object)
  {
    std::lock_guard<std::mutex> lock(visual_mutex_);
    visual_held_ = object;
  }

  bool command_gripper(double position, std::string & reason)
  {
    if (!gripper_client_->wait_for_action_server(10s)) {
      reason = "GRIPPER_UNAVAILABLE";
      return false;
    }
    GripperCommand::Goal goal;
    goal.command.name = {"robotiq_85_left_knuckle_joint"};
    goal.command.position = {position};
    goal.command.velocity = {0.25};
    goal.command.effort = {40.0};
    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(10s) != std::future_status::ready) {
      reason = "GRIPPER_TIMEOUT";
      return false;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      reason = "GRIPPER_REJECTED";
      return false;
    }
    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (result_future.wait_for(15s) != std::future_status::ready) {
      reason = "GRIPPER_TIMEOUT";
      return false;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
      (!wrapped.result->reached_goal && !wrapped.result->stalled))
    {
      reason = "GRIPPER_FAILED";
      return false;
    }
    RCLCPP_INFO(get_logger(), "Robotiq 2F-85 command reached %.3f rad", position);
    return true;
  }

  bool pick(const std::string & object, std::string & reason)
  {
    const auto found = object_poses_.find(object);
    if (found == object_poses_.end()) {
      reason = "INVALID_OBJECT";
      return false;
    }
    if (!held_.empty()) {
      reason = "ALREADY_HOLDING";
      return false;
    }
    auto above = found->second;
    above.position.z += 0.15;
    auto grasp = found->second;
    if (!move_to(above, reason)) {
      return false;
    }
    const auto above_joints = move_group_->getCurrentJointValues();
    if (!command_gripper(0.0, reason)) {
      return false;
    }
    // Contact with the selected cube is intentional. Remove it only for the
    // final descent, then restore it as an attached body after finger closure.
    planning_scene_.removeCollisionObjects({object});
    if (!move_to(grasp, reason, true)) {
      auto upright = upright_pose(found->second);
      planning_scene_.applyCollisionObject(box(object, upright, {0.05, 0.05, 0.05}));
      return false;
    }
    // For a 50 mm cube the official 2F-85 linkage gives a pad opening of
    // approximately 51 mm at 0.35 rad.  Driving farther would force the
    // collision meshes into the static Gazebo cube and can prevent the
    // position controller from ever reporting a completed goal.
    if (!command_gripper(0.35, reason)) {
      auto upright = upright_pose(found->second);
      planning_scene_.applyCollisionObject(box(object, upright, {0.05, 0.05, 0.05}));
      return false;
    }
    auto upright = upright_pose(found->second);
    planning_scene_.applyCollisionObject(box(object, upright, {0.05, 0.05, 0.05}));
    const std::vector<std::string> touch_links = {
      "gripper_tcp", "robotiq_85_base_link", "robotiq_85_left_knuckle_link",
      "robotiq_85_right_knuckle_link", "robotiq_85_left_finger_link",
      "robotiq_85_right_finger_link", "robotiq_85_left_inner_knuckle_link",
      "robotiq_85_right_inner_knuckle_link", "robotiq_85_left_finger_tip_link",
      "robotiq_85_right_finger_tip_link"};
    if (!move_group_->attachObject(object, "gripper_tcp", touch_links)) {
      reason = "ATTACH_FAILED";
      std::string gripper_reason;
      command_gripper(0.0, gripper_reason);
      return false;
    }
    held_ = object;
    set_visual_held(object);
    if (!move_to_joint_values(above_joints, reason)) {
      return false;
    }
    return true;
  }

  bool place(const std::string & object, const std::string & zone, std::string & reason)
  {
    if (held_ != object) {
      reason = "OBJECT_NOT_HELD";
      return false;
    }
    const auto found = zone_poses_.find(zone);
    if (found == zone_poses_.end()) {
      reason = "INVALID_ZONE";
      return false;
    }
    for (const auto & [other, occupied] : locations_) {
      if (other != object && occupied == zone) {
        reason = "ZONE_OCCUPIED";
        return false;
      }
    }
    auto above = found->second;
    above.position.z += 0.15;
    auto release = found->second;
    release.position.z += 0.01;
    if (!move_to(above, reason)) {
      return false;
    }
    const auto above_joints = move_group_->getCurrentJointValues();
    if (!move_to(release, reason, true)) {
      return false;
    }
    if (!command_gripper(0.0, reason)) {
      return false;
    }
    set_visual_held("");
    move_group_->detachObject(object);
    planning_scene_.removeCollisionObjects({object});
    held_.clear();
    auto upright = upright_pose(found->second);
    object_poses_[object] = found->second;
    planning_scene_.applyCollisionObject(box(object, upright, {0.05, 0.05, 0.05}));
    locations_[object] = zone;
    set_gazebo_pose(object, upright);
    return move_to_joint_values(above_joints, reason);
  }

  bool reset(std::string & reason)
  {
    set_visual_held("");
    if (!held_.empty()) {
      move_group_->detachObject(held_);
    }
    if (!command_gripper(0.0, reason)) {
      return false;
    }
    held_.clear();
    if (!home(reason)) {
      return false;
    }
    locations_.clear();
    object_poses_ = initial_object_poses_;
    planning_scene_.removeCollisionObjects({"red_cube", "yellow_cube", "blue_cube"});
    std::vector<moveit_msgs::msg::CollisionObject> cubes;
    for (const auto & [name, object_pose] : object_poses_) {
      auto upright = upright_pose(object_pose);
      cubes.push_back(box(name, upright, {0.05, 0.05, 0.05}));
      set_gazebo_pose(name, upright);
    }
    planning_scene_.applyCollisionObjects(cubes);
    return true;
  }

  void execute(const ExecuteSkill::Request::SharedPtr request, ExecuteSkill::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      initialize();
      std::map<std::string, std::string> args;
      for (const auto & parameter : request->parameters) {
        if (parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
          response->result.successful = false;
          response->result.reason = "INVALID_REQUEST|all arguments must be strings";
          return;
        }
        args[parameter.name] = parameter.value.string_value;
      }
      const auto skill = args["skill"];
      const auto object = args["object"];
      const auto zone = args["zone"];
      std::string status = "SUCCESS";
      bool success = false;
      if (skill == "home") {
        success = home(status);
      } else if (skill == "pick") {
        success = pick(object, status);
      } else if (skill == "place") {
        success = place(object, zone, status);
      } else if (skill == "reset") {
        success = reset(status);
      } else {
        status = "INVALID_SKILL";
      }
      response->result.successful = success;
      response->result.reason = (success ? "SUCCESS" : status) +
        std::string("|") + (success ? "completed" : "skill refused or failed");
    } catch (const std::exception & error) {
      response->result.successful = false;
      response->result.reason = std::string("INTERNAL_ERROR|") + error.what();
      RCLCPP_ERROR(get_logger(), "Skill exception: %s", error.what());
    }
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  const std::map<std::string, geometry_msgs::msg::Pose> initial_object_poses_;
  std::map<std::string, geometry_msgs::msg::Pose> object_poses_;
  std::map<std::string, geometry_msgs::msg::Pose> zone_poses_;
  std::map<std::string, std::string> locations_;
  std::string held_;
  std::string visual_held_;
  bool world_added_{false};
  std::mutex mutex_;
  std::mutex visual_mutex_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Service<ExecuteSkill>::SharedPtr service_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr gz_pose_client_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr follow_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scene_marker_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SkillServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
