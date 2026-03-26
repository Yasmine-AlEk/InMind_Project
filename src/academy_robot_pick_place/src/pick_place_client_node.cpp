#include <chrono>
#include <memory>
#include <string>

#include "academy_robot_interfaces/action/pick_place.hpp"
#include "academy_robot_interfaces/srv/detect_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PickPlaceClientNode : public rclcpp::Node
{
public:
  using DetectObject = academy_robot_interfaces::srv::DetectObject;
  using PickPlace = academy_robot_interfaces::action::PickPlace;
  using GoalHandlePickPlace = rclcpp_action::ClientGoalHandle<PickPlace>;

  PickPlaceClientNode()
  : Node("pick_place_client_node")
  {
    service_name_ = this->declare_parameter<std::string>("service_name", "/detect_object");
    action_name_ = this->declare_parameter<std::string>("action_name", "/pick_place_task");

    wait_timeout_sec_ = this->declare_parameter<double>("wait_timeout_sec", 2.0);
    result_timeout_sec_ = this->declare_parameter<double>("result_timeout_sec", 120.0);

    place_frame_ = this->declare_parameter<std::string>("place_frame", "robot_odom");
    place_x_ = this->declare_parameter<double>("place_x", 6.20);
    place_y_ = this->declare_parameter<double>("place_y", -3.10);
    place_z_ = this->declare_parameter<double>("place_z", 0.40);

    place_qx_ = this->declare_parameter<double>("place_qx", 0.0);
    place_qy_ = this->declare_parameter<double>("place_qy", 0.0);
    place_qz_ = this->declare_parameter<double>("place_qz", 0.0);
    place_qw_ = this->declare_parameter<double>("place_qw", 1.0);

    object_id_ = this->declare_parameter<std::string>("object_id", "socket_cap_screw");

    detect_client_ = this->create_client<DetectObject>(service_name_);
    action_client_ = rclcpp_action::create_client<PickPlace>(this, action_name_);

    RCLCPP_INFO(
      this->get_logger(),
      "T13 client ready | service=%s | action=%s | place_frame=%s | place_xyz=(%.3f, %.3f, %.3f) | object_id=%s | result_timeout=%.1f s",
      service_name_.c_str(),
      action_name_.c_str(),
      place_frame_.c_str(),
      place_x_, place_y_, place_z_,
      object_id_.c_str(),
      result_timeout_sec_);
  }

  int run_once()
  {
    geometry_msgs::msg::PoseStamped detected_pose;

    const int detect_rc = call_detection_service(detected_pose);
    if (detect_rc != 0) {
      return detect_rc;
    }

    return send_goal_and_wait_result(detected_pose);
  }

private:
  int call_detection_service(geometry_msgs::msg::PoseStamped & detected_pose)
  {
    while (!detect_client_->wait_for_service(std::chrono::duration<double>(wait_timeout_sec_))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service %s", service_name_.c_str());
        return 1;
      }
      RCLCPP_WARN(this->get_logger(), "Still waiting for service %s ...", service_name_.c_str());
    }

    auto request = std::make_shared<DetectObject::Request>();

    RCLCPP_INFO(this->get_logger(), "T13 calling /detect_object ...");
    auto future = detect_client_->async_send_request(request);

    const auto result_code =
      rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::seconds(5));

    if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "T13 failed | detect service call did not complete successfully");
      return 1;
    }

    const auto response = future.get();

    if (!response->success) {
      RCLCPP_WARN(this->get_logger(), "T13 detect step returned success=false | no pose available");
      return 2;
    }

    detected_pose = response->detected_pose;

    RCLCPP_INFO(
      this->get_logger(),
      "T13 detect OK | target frame=%s | xyz=(%.4f, %.4f, %.4f)",
      detected_pose.header.frame_id.c_str(),
      detected_pose.pose.position.x,
      detected_pose.pose.position.y,
      detected_pose.pose.position.z);

    return 0;
  }

  void feedback_callback(
    GoalHandlePickPlace::SharedPtr,
    const std::shared_ptr<const PickPlace::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "T13 feedback | current_stage=%s",
      feedback->current_stage.c_str());
  }

  int send_goal_and_wait_result(const geometry_msgs::msg::PoseStamped & detected_pose)
  {
    while (!action_client_->wait_for_action_server(std::chrono::duration<double>(wait_timeout_sec_))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action %s", action_name_.c_str());
        return 3;
      }
      RCLCPP_WARN(this->get_logger(), "Still waiting for action %s ...", action_name_.c_str());
    }

    PickPlace::Goal goal_msg;
    goal_msg.target_pose = detected_pose;
    goal_msg.object_id = object_id_;

    goal_msg.place_pose.header.frame_id = place_frame_;
    goal_msg.place_pose.header.stamp = this->now();
    goal_msg.place_pose.pose.position.x = place_x_;
    goal_msg.place_pose.pose.position.y = place_y_;
    goal_msg.place_pose.pose.position.z = place_z_;
    goal_msg.place_pose.pose.orientation.x = place_qx_;
    goal_msg.place_pose.pose.orientation.y = place_qy_;
    goal_msg.place_pose.pose.orientation.z = place_qz_;
    goal_msg.place_pose.pose.orientation.w = place_qw_;

    RCLCPP_INFO(
      this->get_logger(),
      "T13 sending action goal | pick_frame=%s | pick_xyz=(%.4f, %.4f, %.4f) | place_frame=%s | place_xyz=(%.4f, %.4f, %.4f) | object_id=%s",
      goal_msg.target_pose.header.frame_id.c_str(),
      goal_msg.target_pose.pose.position.x,
      goal_msg.target_pose.pose.position.y,
      goal_msg.target_pose.pose.position.z,
      goal_msg.place_pose.header.frame_id.c_str(),
      goal_msg.place_pose.pose.position.x,
      goal_msg.place_pose.pose.position.y,
      goal_msg.place_pose.pose.position.z,
      goal_msg.object_id.c_str());

    rclcpp_action::Client<PickPlace>::SendGoalOptions options;
    options.feedback_callback =
      std::bind(&PickPlaceClientNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    auto future_goal_handle = action_client_->async_send_goal(goal_msg, options);

    const auto goal_code =
      rclcpp::spin_until_future_complete(shared_from_this(), future_goal_handle, std::chrono::seconds(5));

    if (goal_code != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "T13 failed | action goal request did not complete");
      return 4;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "T13 failed | action goal was rejected");
      return 5;
    }

    RCLCPP_INFO(this->get_logger(), "T13 goal accepted | waiting for final result ...");

    auto future_result = action_client_->async_get_result(goal_handle);

    const auto result_code =
      rclcpp::spin_until_future_complete(
        shared_from_this(),
        future_result,
        std::chrono::duration<double>(result_timeout_sec_));

    if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "T13 failed | timed out or interrupted while waiting for final result");
      return 6;
    }

    const auto wrapped_result = future_result.get();

    const char * status_text = "UNKNOWN";
    switch (wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status_text = "SUCCEEDED";
        break;
      case rclcpp_action::ResultCode::ABORTED:
        status_text = "ABORTED";
        break;
      case rclcpp_action::ResultCode::CANCELED:
        status_text = "CANCELED";
        break;
      default:
        status_text = "UNKNOWN";
        break;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "T13 result | status=%s | success=%s | planning_time_ms=%ld | execution_time_ms=%ld",
      status_text,
      wrapped_result.result->success ? "true" : "false",
      wrapped_result.result->planning_time_ms,
      wrapped_result.result->execution_time_ms);

    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "T13 OK | action finished successfully");
      return 0;
    }

    RCLCPP_WARN(this->get_logger(), "T13 complete | action finished but did not succeed");
    return 7;
  }

  std::string service_name_;
  std::string action_name_;
  double wait_timeout_sec_;
  double result_timeout_sec_;

  std::string place_frame_;
  double place_x_;
  double place_y_;
  double place_z_;
  double place_qx_;
  double place_qy_;
  double place_qz_;
  double place_qw_;

  std::string object_id_;

  rclcpp::Client<DetectObject>::SharedPtr detect_client_;
  rclcpp_action::Client<PickPlace>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceClientNode>();
  const int rc = node->run_once();
  rclcpp::shutdown();
  return rc;
}
