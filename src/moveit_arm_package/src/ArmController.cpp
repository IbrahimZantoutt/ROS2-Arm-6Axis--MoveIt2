#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <thread>


class ArmController : public rclcpp::Node {
public:
  ArmController() : Node("arm_controller") {
    // Depth-1 "keep last": the vision node publishes a target on every camera
    // frame (~30 Hz). We only ever care about the most recent one, so the
    // callback just stashes it and a single worker thread consumes it serially.
    target_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>
      ("object_position", rclcpp::QoS(1),
       std::bind(&ArmController::targetCallback, this, std::placeholders::_1));
  }

  ~ArmController() override {
    stop_ = true;
    if (worker_.joinable()) worker_.join();
  }

  void init() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setMaxVelocityScalingFactor(0.5);
    move_group_interface_->setMaxAccelerationScalingFactor(0.5);

    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");
  }

  // Start the worker thread that consumes the latest target. Must be called
  // after init() and once the executor is (about to be) spinning, since the
  // worker blocks on the compute_ik future which the executor delivers.
  void start() {
    worker_ = std::thread([this]() { workerLoop(); });
  }

  // Returns true only if a collision-free IK solution exists for `target`.
  // Logs the reason when it does not, so a failed pose is explained rather than
  // silently aborting later during planning.
  bool isPoseReachable(const geometry_msgs::msg::Pose& target) {
    if (!ik_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "compute_ik service unavailable; skipping pre-check");
      return true;  // fall through to planning rather than blocking on the check
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = "arm";
    request->ik_request.avoid_collisions = true;
    request->ik_request.pose_stamped.header.frame_id =
      move_group_interface_->getPlanningFrame();
    request->ik_request.pose_stamped.pose = target;
    request->ik_request.timeout = rclcpp::Duration::from_seconds(2.0);

    // We're on the worker thread (not an executor callback), so don't spin the
    // node. Just wait on the future; the executor running in main() delivers the
    // service response.
    auto future = ik_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "compute_ik call timed out; skipping pre-check");
      return true;
    }

    const int code = future.get()->error_code.val;
    if (code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      return true;
    }

    const char* reason =
      code == moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION
        ? "no collision-free IK solution (pose unreachable or every solution self-collides)"
      : code == moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION
        ? "goal is in collision"
      : code == moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME
        ? "invalid group name"
        : "IK failed";
    RCLCPP_ERROR(get_logger(), "Pose rejected: %s (error_code=%d)", reason, code);
    return false;
  }

  void moveToPose(double x, double y, double z,
                  double qx = 0.0, double qy = 0.7071, double qz = 0.0, double qw = 0.7071) {
    geometry_msgs::msg::Pose target;
    target.position.x = x;
    target.position.y = y;
    target.position.z = z;
    target.orientation.x = qx;
    target.orientation.y = qy;
    target.orientation.z = qz;
    target.orientation.w = qw;

    if (!isPoseReachable(target)) {
      RCLCPP_INFO(get_logger(), "Skipping planning to (%.2f, %.2f, %.2f)", x, y, z);
      return;
    }

    move_group_interface_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(get_logger(), "Planning to (%.2f, %.2f, %.2f): %s",
      x, y, z, success ? "SUCCEEDED" : "FAILED");

    if (success) {
      move_group_interface_->execute(plan);
    }
  }

  void moveToNamed(const std::string& pose_name) {
    move_group_interface_->setNamedTarget(pose_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(get_logger(), "Planning to '%s': %s",
      pose_name.c_str(), success ? "SUCCEEDED" : "FAILED");

    if (success) {
      move_group_interface_->execute(plan);
    }
  }

  // Cheap and non-blocking: just record the most recent target. Older unhandled
  // targets are overwritten, so we never queue up stale motions.
  void targetCallback(geometry_msgs::msg::PointStamped::SharedPtr msg){
    std::lock_guard<std::mutex> lock(target_mutex_);
    latest_target_ = msg->point;
  }

  // Runs in its own thread so MoveGroupInterface (which is NOT thread-safe) is
  // only ever touched from here, one motion at a time.
  void workerLoop() {
    using namespace std::chrono_literals;
    while (rclcpp::ok() && !stop_) {
      std::optional<geometry_msgs::msg::Point> target;
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (latest_target_) {
          target = latest_target_;
          latest_target_.reset();  // consume it
        }
      }

      if (!target) {
        std::this_thread::sleep_for(50ms);
        continue;
      }

      RCLCPP_INFO(get_logger(), "Processing target: x=%.3f y=%.3f z=%.3f",
        target->x, target->y, target->z);
      // For simplicity, use a fixed orientation (e.g., end-effector pointing down).
      moveToPose(target->x, target->y, target->z);
    }
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;

  std::mutex target_mutex_;
  std::optional<geometry_msgs::msg::Point> latest_target_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmController>();
  node->init();
  
  // relies on the same spinning executor, so this works for both modes.
  std::thread spin_thread([node]() { rclcpp::spin(node); });
  // Later: enable the camera-driven worker instead of the direct test call above.
  node->start();
  
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
