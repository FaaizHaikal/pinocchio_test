#ifndef PINOCCHIO_TEST__NODE__PINOCCHIO_TEST_NODE_HPP_
#define PINOCCHIO_TEST__NODE__PINOCCHIO_TEST_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "kansei_interfaces/msg/status.hpp"
#include "pinocchio_test/model/robot_wrapper.hpp"

namespace pinocchio_test
{

class PinocchioTestNode
{
public:
  using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;
  using KanseiStatus = kansei_interfaces::msg::Status;

  PinocchioTestNode(const rclcpp::Node::SharedPtr & node, const std::string & urdf_path);

  void publish_robot_state();

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<RobotWrapper> robot_wrapper;
  rclcpp::Subscription<CurrentJoints>::SharedPtr current_joints_subscriber;
  rclcpp::Subscription<KanseiStatus>::SharedPtr kansei_status_subscriber;

};

}  // namespace pinocchio_test

#endif  // PINOCCHIO_TEST__NODE__PINOCCHIO_TEST_NODE_HPP_
