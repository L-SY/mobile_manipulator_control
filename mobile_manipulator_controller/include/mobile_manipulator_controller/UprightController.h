//
// Created by lsy on 24-3-6.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <position_controllers/joint_group_position_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <robot_common/interface/hardware_interface/HybridJointInterface.h>
#include <control_toolbox/pid.h>
#include <robot_common/utilities/lp_filter.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ros/package.h>
#include <mobile_manipulator_interface/MobileManipulatorInterface.h>

#include "definitions.h"
#include "mobile_manipulator_controller/MobileManipulatorVisualization.h"
#include "mobile_manipulator_controller/synchronized_module/RosReferenceManager.h"
#include <robot_common/utilities/ori_tool.h>
#include <robot_common/utilities/tf_rt_broadcaster.h>

// Use for gravity compensation
#include <arm_pinocchio_interface/EndEffectorInterface.h>
#include <arm_pinocchio_interface/PinocchioInterface.h>
#include <arm_pinocchio_interface/urdf.h>

// Interpolation
#include <mobile_manipulator_controller/interpolation/MultiJointTrajectory.h>

// Realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace ddt
{
struct TimeData
{
  TimeData() : time(0.0), period(0.0), uptime(0.0)
  {
  }

  ros::Time time;        ///< Time of last update cycle
  ros::Duration period;  ///< Period of last update cycle
  ros::Time uptime;      ///< Controller uptime. Set to zero at every restart.
};

class UprightController
  : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                          hardware_interface::EffortJointInterface,hardware_interface::HybridJointInterface>
{
  enum ControllerState
  {
    NORMAL,         // Only try to achieve EE to reach a specified point in space
    UPRIGHT,        // On the basis of normal, increase the importance of keeping the
                    // objects on EE upright
    UPRIGHT_AVOID,  // On the basis of upright, increase the function of avoid
                    // obstacle
    EE_SPACE_LOCK   // Try to lock the EE on a space point while move the  robot
                    // base
  };

public:
  UprightController() = default;

  ~UprightController() override;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void starting(const ros::Time& time) override;

  void stopping(const ros::Time& /*time*/) override
  {
    mpcRunning_ = false;
  }

protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  ocs2_msgs::mpc_flattened_controller createMpcPolicyMsg(const ocs2::PrimalSolution& primalSolution,
                                                         const ocs2::CommandData& commandData,
                                                         const ocs2::PerformanceIndex& performanceIndices);

  void updateTfOdom(const ros::Time& time, const ros::Duration& period);

  void normal(const ros::Time& time, const ros::Duration& period);

  void upright(const ros::Time& time, const ros::Duration& period);

  void uprightAvoid(const ros::Time& time, const ros::Duration& period);

  void EESpaceLock(const ros::Time& time, const ros::Duration& period);

  void worldV2BaseV();

  virtual void setupMpc(ros::NodeHandle& nh);

  virtual void setupMrt();

  // Interface
  std::shared_ptr<upright::ControllerInterface> mobileManipulatorInterface_;
  std::vector<hardware_interface::JointHandle> jointHandles_;

  // State Estimation
  ocs2::SystemObservation currentObservation_, lastObservation_;
  bool updatePolicy_ = false;
  //only use in gazebo
  std::vector<LowPassFilter> velLPFs_;

  // Nonlinear MPC
  std::unique_ptr<ocs2::MultipleShootingMpc> mpc_;
  std::shared_ptr<ocs2::MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<ddt::MobileManipulatorDummyVisualization> visualizer_;
  ros::Publisher observationPublisher_;

private:
  int controlState_ = UPRIGHT;
  std::thread mpcThread_;

  ros::Publisher optimizedStateTrajectoryPub_, gravityCompPub_, velLPFPub_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  ocs2::benchmark::RepeatedTimer mpcTimer_;

  // Odom TF
  tf::TfRtBroadcaster tfRtBroadcaster_;
  geometry_msgs::TransformStamped odom2base_{};

  ocs2::vector_t jointVelLast_{};
  ros::Time lastTime_{};

  double init_x_, init_y_, init_z_, yaw_;
  std::vector<hardware_interface::HybridJointHandle> hybridJointHandles_;
  std::vector<hardware_interface::JointHandle> effortJointHandles_;
  std::vector<hardware_interface::JointHandle> velocityJointHandles_;
  std::vector<hardware_interface::JointHandle> wheelJointHandles_;
  // Use for gravity compensation
  std::shared_ptr<arm_pinocchio::PinocchioInterface> pinocchioInterface_;
  std::shared_ptr<arm_pinocchio::EndEffectorInterface<double>> endEffectorInterface_;

  // Low level controller
  std::vector<control_toolbox::Pid> pids_;
  position_controllers::JointGroupPositionController armPositionController_;
  // interpolation
  std::shared_ptr<interpolation::MultiJointTrajectory> splineTrajectory_;

  // Iner time
  realtime_tools::RealtimeBuffer<TimeData> time_data_;
  TimeData old_time_data_;
};
}  // namespace ddt