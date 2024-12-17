#include "pinocchio_test/node/pinocchio_test_node.hpp"

using namespace std::chrono_literals;

namespace pinocchio_test
{

PinocchioTestNode::PinocchioTestNode(
  const rclcpp::Node::SharedPtr & node, const std::string & urdf_path)
: node(node), robot_wrapper(std::make_shared<RobotWrapper>(urdf_path))
{
  current_joints_subscriber = node->create_subscription<CurrentJoints>(
    "current_joints", 10, [this](const CurrentJoints::SharedPtr msg) {
      for (const auto & joint : msg->joints) {
        robot_wrapper->update_joint_position(joint.id, joint.position);
      }
    });

  kansei_status_subscriber = node->create_subscription<KanseiStatus>(
    "kansei_status", 10, [this](const KanseiStatus::SharedPtr msg) {
      auto orientation = msg->orientation;
      robot_wrapper->update_imu_link(orientation.roll, orientation.pitch, orientation.yaw);
    });

  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  node_timer = node->create_wall_timer(8ms, [this]() { this->publish_robot_state(); });
}

void PinocchioTestNode::publish_robot_state()
{
  auto frames = robot_wrapper->get_tf_frames();

  for (auto & frame : frames) {
    frame.header.stamp = node->now();  // Update timestamp
    tf_broadcaster->sendTransform(frame);
  }
}

}  // namespace pinocchio_test
