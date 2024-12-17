#include "pinocchio_test/model/robot_wrapper.hpp"

namespace pinocchio_test
{

RobotWrapper::RobotWrapper(const std::string & urdf_path)
{
  pinocchio::urdf::buildModel(urdf_path, model);
  data = pinocchio::Data(model);

  reset_joints();
}

void RobotWrapper::update_joint_position(u_int8_t joint_id, double position)
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  q[joint_id - 1] = position;

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
}

void RobotWrapper::update_imu_link(double roll, double pitch, double yaw)
{
  Eigen::Vector3d rpy(roll, pitch, yaw);
  pinocchio::SE3 imu_pose = pinocchio::SE3(pinocchio::Rotation::RPY(rpy));

  data.oMi[model.getFrameId("imu_link")] = imu_pose;

  pinocchio::updateFramePlacements(model, data);
} // namespace pinocchio_test

void RobotWrapper::reset_joints()
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
}

} // namespace pinocchio_test
