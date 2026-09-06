#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

class TargetPoseSubscriber : public rclcpp::Node
{
public:
  explicit TargetPoseSubscriber(const rclcpp::NodeOptions & options)
  : Node("target_pose_subscriber", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
  }

  void initialize()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "panda_arm", std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration::from_seconds(10.0));
    planning_frame_ = move_group_->getPlanningFrame();
    RCLCPP_INFO(get_logger(), "MoveIt planning frame: '%s'", planning_frame_.c_str());
    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);

    // Serialize targets while allowing MoveIt callbacks to run on another thread.
    target_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = target_callback_group_;
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        if (message->header.frame_id.empty()) {
          RCLCPP_WARN(get_logger(), "Ignoring target pose: header.frame_id is empty");
          return;
        }

        geometry_msgs::msg::PoseStamped transformed_pose;
        if (message->header.frame_id == planning_frame_) {
          transformed_pose = *message;
        } else {
          try {
            transformed_pose = tf_buffer_.transform(*message, planning_frame_);
          } catch (const tf2::TransformException & exception) {
            RCLCPP_WARN(
              get_logger(), "Could not transform target pose from '%s' to '%s': %s",
              message->header.frame_id.c_str(), planning_frame_.c_str(), exception.what());
            return;
          }
        }

        const auto & position = transformed_pose.pose.position;
        const auto & orientation = transformed_pose.pose.orientation;
        RCLCPP_INFO(
          get_logger(),
          "original_frame='%s', planning_frame='%s', position: x=%.6f y=%.6f z=%.6f, "
          "orientation: x=%.6f y=%.6f z=%.6f w=%.6f",
          message->header.frame_id.c_str(),
          planning_frame_.c_str(),
          position.x, position.y, position.z,
          orientation.x, orientation.y, orientation.z, orientation.w);

        move_group_->setStartStateToCurrentState();
        if (!move_group_->setPoseTarget(transformed_pose)) {
          RCLCPP_WARN(get_logger(), "Planning failed: pose target was rejected; plan not requested");
          move_group_->clearPoseTargets();
          return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto start = std::chrono::steady_clock::now();
        bool planning_succeeded = false;
        try {
          const auto result = move_group_->plan(plan);
          planning_succeeded = result == moveit::core::MoveItErrorCode::SUCCESS;
          const double latency_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
          RCLCPP_INFO(
            get_logger(), "Planning %s: error_code=%d, latency_ms=%.3f",
            planning_succeeded ? "succeeded" : "failed",
            result.val, latency_ms);
        } catch (const std::exception & exception) {
          const double latency_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
          RCLCPP_WARN(
            get_logger(), "Planning failed: %s, latency_ms=%.3f",
            exception.what(), latency_ms);
        }

        if (planning_succeeded) {
          const auto execution_start = std::chrono::steady_clock::now();
          try {
            const auto result = move_group_->execute(plan);
            const double execution_latency_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - execution_start).count();
            if (result == moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_INFO(
                get_logger(), "Execution succeeded: error_code=%d, latency_ms=%.3f",
                result.val, execution_latency_ms);
            } else {
              RCLCPP_WARN(
                get_logger(), "Execution failed: error_code=%d, latency_ms=%.3f",
                result.val, execution_latency_ms);
            }
          } catch (const std::exception & exception) {
            const double execution_latency_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - execution_start).count();
            RCLCPP_WARN(
              get_logger(), "Execution failed: %s, error_code=unavailable, latency_ms=%.3f",
              exception.what(), execution_latency_ms);
          }
        } else {
          RCLCPP_INFO(get_logger(), "Execution skipped: planning failed");
        }
        const double total_latency_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
        RCLCPP_INFO(get_logger(), "Total plan + execute latency_ms=%.3f", total_latency_ms);
        move_group_->clearPoseTargets();
      }, subscription_options);
    RCLCPP_INFO(get_logger(), "Ready to plan and execute for panda_arm (velocity/acceleration scaling=0.1)");
  }

  void release_move_group()
  {
    // MoveGroupInterface retains a shared pointer to this node.
    move_group_.reset();
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string planning_frame_;
  rclcpp::CallbackGroup::SharedPtr target_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<TargetPoseSubscriber>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  int exit_code = 0;
  try {
    node->initialize();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(node->get_logger(), "Node failed: %s", exception.what());
    exit_code = 1;
  }
  node->release_move_group();
  rclcpp::shutdown();
  return exit_code;
}
