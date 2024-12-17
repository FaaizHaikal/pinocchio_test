#ifndef PINOCCHIO_TEST__MODEL__ROBOT_WRAPPER_HPP
#define PINOCCHIO_TEST__MODEL__ROBOT_WRAPPER_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <string>

namespace pinocchio_test
{

class RobotWrapper
{
public:
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  RobotWrapper(const std::string & urdf_path);
  void update_joint_position(u_int8_t joint_id, double position);
  void update_imu_link(double roll, double pitch, double yaw);

  void reset_joints();

  const std::vector<TransformStamped> & get_tf_frames();

private:
  pinocchio::Model model;
  pinocchio::Data data;
};

}  // namespace pinocchio_test

#endif  // PINOCCHIO_TEST__MODEL__ROBOT_WRAPPER_HPP
