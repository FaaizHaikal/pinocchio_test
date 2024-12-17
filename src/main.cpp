#include "pinocchio_test/pinocchio_test.hpp"

int main(int argc, char ** argv)
{
  std::string urdf_path = argv[1];

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("pinocchio_test_node");
  auto pinocchio_test_node = std::make_shared<pinocchio_test::PinocchioTestNode>(node, urdf_path);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}