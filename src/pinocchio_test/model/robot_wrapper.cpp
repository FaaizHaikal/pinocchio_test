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
  pinocchio::updateGlobalPlacements(model, data);
}

void RobotWrapper::update_imu_link(double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond quat = yaw_angle * pitch_angle * roll_angle;

  const std::string imu_link_name = "imu_link";
  if (model.existFrame(imu_link_name)) {
    const auto & imu_link_id = model.getFrameId(imu_link_name);
    data.oMf[imu_link_id].rotation() = quat.toRotationMatrix();
  } else {
    std::cerr << "IMU link not found in model" << std::endl;
  }
}

void RobotWrapper::reset_joints()
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateGlobalPlacements(model, data);
}

const std::vector<RobotWrapper::TransformStamped> & RobotWrapper::get_tf_frames()
{
  std::vector<TransformStamped> tf_frames;

  for (size_t i = 0; i < model.frames.size(); i++) {
    const auto & frame = model.frames[i];
    const auto & transform = data.oMf[i];

    TransformStamped tf_frame;
    tf_frame.header.frame_id = model.frames[frame.parent].name;
    tf_frame.child_frame_id = frame.name;
    tf_frame.transform.translation.x = transform.translation()(0);
    tf_frame.transform.translation.y = transform.translation()(1);
    tf_frame.transform.translation.z = transform.translation()(2);

    Eigen::Quaterniond quat(transform.rotation());
    tf_frame.transform.rotation.x = quat.x();
    tf_frame.transform.rotation.y = quat.y();
    tf_frame.transform.rotation.z = quat.z();
    tf_frame.transform.rotation.w = quat.w();

    tf_frames.push_back(tf_frame);
  }

  return tf_frames;
}

}  // namespace pinocchio_test
