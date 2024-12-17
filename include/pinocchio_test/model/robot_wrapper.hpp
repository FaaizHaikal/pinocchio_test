#ifndef PINOCCHIO_TEST__MODEL__ROBOT_WRAPPER_HPP
#define PINOCCHIO_TEST__MODEL__ROBOT_WRAPPER_HPP

#include <string>
#include <pinocchio/pinocchio.hpp>

namespace pinocchio_test
{

class RobotWrapper
{
  public:
    RobotWrapper(const std::string & urdf_path);
    void update_joint_position(u_int8_t joint_id, double position);
    void update_imu_link(double roll, double pitch, double yaw);

    void reset_joints();

  private:
    pinocchio::Model model;
    pinocchio::Data data;
}

} // namespace pinocchio_test

#endif // PINOCCHIO_TEST__MODEL__ROBOT_WRAPPER_HPP
