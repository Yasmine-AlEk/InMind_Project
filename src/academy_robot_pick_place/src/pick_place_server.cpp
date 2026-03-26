#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "academy_robot_interfaces/action/pick_place.hpp"
#include "moveit/task_constructor/solvers.h"
#include "moveit/task_constructor/stages.h"
#include "moveit/task_constructor/task.h"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mtc = moveit::task_constructor;

class PickPlaceServer : public rclcpp::Node
{
public:
  using PickPlace = academy_robot_interfaces::action::PickPlace;
  using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;

  PickPlaceServer()
  : Node("pick_place_server")
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "/pick_place_task");
    mtc_task_name_ = this->declare_parameter<std::string>("mtc_task_name", "pick_place_task");

    arm_group_name_ = this->declare_parameter<std::string>("arm_group_name", "ur_arm");
    arm_pregrasp_pose_name_ =
      this->declare_parameter<std::string>("arm_pregrasp_pose_name", "pregrasp");
    arm_place_pregrasp_pose_name_ =
      this->declare_parameter<std::string>("arm_place_pregrasp_pose_name", "place_pregrasp");

    hand_group_name_ = this->declare_parameter<std::string>("hand_group_name", "hand");
    hand_open_pose_name_ = this->declare_parameter<std::string>("hand_open_pose_name", "open");
    hand_close_pose_name_ = this->declare_parameter<std::string>("hand_close_pose_name", "close");

    max_solutions_ = this->declare_parameter<int>("max_solutions", 1);

    action_server_ = rclcpp_action::create_server<PickPlace>(
      this,
      action_name_,
      std::bind(&PickPlaceServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::handleCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "T10 server ready | action=%s | mtc_task_name=%s | arm_group=%s | arm_pregrasp_pose=%s | arm_place_pregrasp_pose=%s | hand_group=%s | hand_open_pose=%s | hand_close_pose=%s | max_solutions=%d",
      action_name_.c_str(),
      mtc_task_name_.c_str(),
      arm_group_name_.c_str(),
      arm_pregrasp_pose_name_.c_str(),
      arm_place_pregrasp_pose_name_.c_str(),
      hand_group_name_.c_str(),
      hand_open_pose_name_.c_str(),
      hand_close_pose_name_.c_str(),
      max_solutions_);
  }

private:
  void publishStageFeedback(
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle,
    const std::string & stage_name)
  {
    auto feedback = std::make_shared<PickPlace::Feedback>();
    feedback->current_stage = stage_name;
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(
      this->get_logger(),
      "T10 feedback | current_stage=%s",
      stage_name.c_str());
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickPlace::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received PickPlace goal | object_id=%s | target_frame=%s | place_frame=%s",
      goal->object_id.c_str(),
      goal->target_pose.header.frame_id.c_str(),
      goal->place_pose.header.frame_id.c_str());

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePickPlace>)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request for PickPlace action");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    std::thread{std::bind(&PickPlaceServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  bool buildStagePipeline(
    const std::shared_ptr<const PickPlace::Goal> & goal,
    mtc::Task & task,
    const std::shared_ptr<GoalHandlePickPlace> & goal_handle)
  {
    (void)goal;

    try {
      publishStageFeedback(goal_handle, "task_initializing");

      task.stages()->setName("root");
      task.setName(mtc_task_name_);
      task.loadRobotModel(shared_from_this());

      auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

      publishStageFeedback(goal_handle, "adding_current_state");
      {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        task.add(std::move(stage));
      }

      publishStageFeedback(goal_handle, "adding_pregrasp");
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>(
          "move arm to pregrasp",
          interpolation_planner);
        stage->setGroup(arm_group_name_);
        stage->setGoal(arm_pregrasp_pose_name_);
        task.add(std::move(stage));
      }

      publishStageFeedback(goal_handle, "adding_open_hand_pregrasp");
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>(
          "open hand before grasp",
          interpolation_planner);
        stage->setGroup(hand_group_name_);
        stage->setGoal(hand_open_pose_name_);
        task.add(std::move(stage));
      }

      publishStageFeedback(goal_handle, "adding_place_pregrasp");
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>(
          "move arm to place pregrasp",
          interpolation_planner);
        stage->setGroup(arm_group_name_);
        stage->setGoal(arm_place_pregrasp_pose_name_);
        task.add(std::move(stage));
      }

      publishStageFeedback(goal_handle, "adding_open_hand_release");
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>(
          "open hand to release",
          interpolation_planner);
        stage->setGroup(hand_group_name_);
        stage->setGoal(hand_open_pose_name_);
        task.add(std::move(stage));
      }

      publishStageFeedback(goal_handle, "pipeline_built");

      RCLCPP_INFO(
        this->get_logger(),
        "T10 pipeline built | stages=current state, move arm to pregrasp, open hand, move arm to place pregrasp, open hand");

      return true;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "T10 stage construction failed: %s",
        ex.what());
      return false;
    }
  }

  void execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    auto result = std::make_shared<PickPlace::Result>();
    result->success = false;
    result->planning_time_ms = 0;
    result->execution_time_ms = 0;

    publishStageFeedback(goal_handle, "goal_received");

    mtc::Task task;
    const bool built = buildStagePipeline(goal, task, goal_handle);

    if (!built) {
      publishStageFeedback(goal_handle, "pipeline_build_failed");
      goal_handle->abort(result);
      return;
    }

    try {
      publishStageFeedback(goal_handle, "pipeline_initializing");
      task.init();
      publishStageFeedback(goal_handle, "pipeline_initialized");
    } catch (const mtc::InitStageException & ex) {
      std::ostringstream oss;
      oss << ex;
      RCLCPP_ERROR(
        this->get_logger(),
        "T10 task.init() failed:\n%s",
        oss.str().c_str());

      publishStageFeedback(goal_handle, "pipeline_init_failed");
      goal_handle->abort(result);
      return;
    }

    publishStageFeedback(goal_handle, "planning_started");
    const auto planning_start = std::chrono::steady_clock::now();
    const auto planning_result = task.plan(max_solutions_);
    const auto planning_end = std::chrono::steady_clock::now();
    result->planning_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(planning_end - planning_start).count();

    if (planning_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || task.solutions().empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "T10 planning failed | error_code=%d | planning_time_ms=%ld",
        planning_result.val,
        result->planning_time_ms);

      publishStageFeedback(goal_handle, "planning_failed");
      goal_handle->abort(result);
      return;
    }

    publishStageFeedback(goal_handle, "planning_succeeded");
    task.introspection().publishSolution(*task.solutions().front());

    publishStageFeedback(goal_handle, "execution_started");
    const auto execution_start = std::chrono::steady_clock::now();
    const auto exec_result = task.execute(*task.solutions().front());
    const auto execution_end = std::chrono::steady_clock::now();
    result->execution_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(execution_end - execution_start).count();

    if (exec_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "T10 execution failed | error_code=%d | planning_time_ms=%ld | execution_time_ms=%ld",
        exec_result.val,
        result->planning_time_ms,
        result->execution_time_ms);

      publishStageFeedback(goal_handle, "execution_failed");
      goal_handle->abort(result);
      return;
    }

    result->success = true;
    publishStageFeedback(goal_handle, "execution_succeeded");

    RCLCPP_INFO(
      this->get_logger(),
      "T10 OK | plan() and execute() succeeded | planning_time_ms=%ld | execution_time_ms=%ld",
      result->planning_time_ms,
      result->execution_time_ms);

    goal_handle->succeed(result);
  }

  std::string action_name_;
  std::string mtc_task_name_;

  std::string arm_group_name_;
  std::string arm_pregrasp_pose_name_;
  std::string arm_place_pregrasp_pose_name_;

  std::string hand_group_name_;
  std::string hand_open_pose_name_;
  std::string hand_close_pose_name_;

  int max_solutions_;

  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
