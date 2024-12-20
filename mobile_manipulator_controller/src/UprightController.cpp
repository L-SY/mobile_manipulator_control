//
// Created by lsy on 24-3-6.
//

#include "mobile_manipulator_controller/UprightController.h"

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <pluginlib/class_list_macros.h>

namespace ddt
{

bool UprightController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  // Initialize OCS2
  std::string taskFile;
  std::string libFolder;
  std::string urdfFile;
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/libFolder", libFolder);
  controller_nh.getParam("/urdfFile", urdfFile);
  mobileManipulatorInterface_ = std::make_shared<upright::ControllerInterface>(taskFile, libFolder, urdfFile);
  setupMpc(controller_nh);
  setupMrt();

  // Visualization
  ros::NodeHandle nh;
  visualizer_ = std::make_shared<ddt::MobileManipulatorDummyVisualization>(nh, *mobileManipulatorInterface_);

  currentObservation_.time = 0.0;
  currentObservation_.state.setZero(STATE_DIM);
  currentObservation_.input.setZero(INPUT_DIM);
  lastObservation_ = currentObservation_;

  double lp_cutoff_frequency;
  if (!controller_nh.getParam("lp_cutoff_frequency", lp_cutoff_frequency))
  {
    lp_cutoff_frequency = 100;
  }
  for (int i = 0; i < 6; ++i) {
    velLPFs_.emplace_back(lp_cutoff_frequency);
  }
  jointVelLast_.resize(10);
  lastTime_ = ros::Time::now();
  // Hardware interface
  auto* velocityJointInterface = robot_hw->get<hardware_interface::VelocityJointInterface>();
  velocityJointHandles_.push_back(velocityJointInterface->getHandle("baseX"));
  velocityJointHandles_.push_back(velocityJointInterface->getHandle("baseY"));
  velocityJointHandles_.push_back(velocityJointInterface->getHandle("baseYaw"));
  wheelJointHandles_.push_back(velocityJointInterface->getHandle("left_j3"));
  wheelJointHandles_.push_back(velocityJointInterface->getHandle("right_j3"));

  auto* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
  auto* hybridJointInterface = robot_hw->get<hardware_interface::HybridJointInterface>();
  std::vector<std::string> swingboyJointNames{ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

  for (const auto& joint_name : swingboyJointNames)
  {
//    effortJointHandles_.push_back(effortJointInterface->getHandle(joint_name));
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }

  controlState_ = UPRIGHT;
  // Odom TF
  odom2base_.header.frame_id = "world";
  odom2base_.header.stamp = ros::Time::now();
  odom2base_.child_frame_id = "diablo_base_link";

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, 0);
  odom2base_.transform.rotation.x = quaternion.x();
  odom2base_.transform.rotation.y = quaternion.y();
  odom2base_.transform.rotation.z = quaternion.z();
  odom2base_.transform.rotation.w = quaternion.w();

  odom2base_.transform.translation.x = 0;
  odom2base_.transform.translation.y = 0;
  odom2base_.transform.translation.z = 0;
  tfRtBroadcaster_.init(root_nh);
  tfRtBroadcaster_.sendTransform(odom2base_);

  controller_nh.getParam("init_pos/x", init_x_);
  controller_nh.getParam("init_pos/y", init_y_);
  controller_nh.getParam("init_pos/z", init_z_);

  optimizedStateTrajectoryPub_ =
      nh.advertise<ocs2_msgs::mpc_flattened_controller>("/mobile_manipulator_mpc_policy", 10);
  ROS_INFO_STREAM("UprightController Init Finish!");
  // Use for gravity compensation
  std::string armURDFFile;
  controller_nh.getParam("arm_urdf", armURDFFile);
  std::vector<std::string> endEffectorName = { "link6" };
  pinocchioInterface_ = std::make_shared<arm_pinocchio::PinocchioInterface>(
      arm_pinocchio::getPinocchioInterfaceFromUrdfFile(armURDFFile));
  //  std::cout << *pinocchioInterface_ << std::endl;
  endEffectorInterface_ =
      std::make_shared<arm_pinocchio::EndEffectorInterface<double>>(*pinocchioInterface_, endEffectorName);
  endEffectorInterface_->setPinocchioInterface(*pinocchioInterface_);
  gravityCompPub_ = nh.advertise<std_msgs::Float64MultiArray>("/gravity_compensation", 10);
  velLPFPub_ = nh.advertise<std_msgs::Float64MultiArray>("/vel_lowpass_out", 10);
// Low level controller
// Use for MIT Motor controller
  std::vector<std::string> joint_names = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
  pids_.resize(joint_names.size());
  for (unsigned int i = 0; i < pids_.size(); ++i)
  {
    ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_names[i]);

    pids_[i].reset();
    if (!pids_[i].init(joint_nh))
    {
      ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
      return false;
    }
  }
  return true;
}

void UprightController::update(const ros::Time& time, const ros::Duration& period)
{
  old_time_data_ = *(time_data_.readFromRT());
  // Update time data
  TimeData time_data;
  time_data.time = time;                              // Cache current time
  time_data.period = period;                          // Cache current control period
  time_data.uptime = old_time_data_.uptime + period;  // Update controller uptime
  time_data_.writeFromNonRT(time_data);

  // Update the current state of the system
  updateTfOdom(time, period);
  updateStateEstimation(time, period);
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  std::vector<double> q;
  std::vector<double> v;
//  for (auto& effortJointHandle : effortJointHandles_)
//  {
//    q.push_back(effortJointHandle.getPosition());
//    v.push_back(effortJointHandle.getVelocity());
//  }
  for (auto& hybridJointHandle : hybridJointHandles_)
  {
    q.push_back(hybridJointHandle.getPosition());
    v.push_back(hybridJointHandle.getVelocity());
  }
  Eigen::VectorXd q_eigen = Eigen::VectorXd::Map(q.data(), q.size());
  Eigen::VectorXd v_eigen = Eigen::VectorXd::Map(v.data(), v.size());
  endEffectorInterface_->update(q_eigen, v_eigen);
  switch (controlState_)
  {
    case ControllerState::NORMAL:
      normal(time, period);
      break;
    case ControllerState::UPRIGHT:  // Add when normal is useful
      upright(time, period);
      break;
    case ControllerState::UPRIGHT_AVOID:  // Add when normal is useful
      uprightAvoid(time, period);
      break;
    case ControllerState::EE_SPACE_LOCK:  // Add when normal is useful
      EESpaceLock(time, period);
      break;
  }

  // Visualization
  visualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ocs2::ros_msg_conversions::createObservationMsg(currentObservation_));
  lastTime_ = ros::Time::now();
}

void UprightController::updateTfOdom(const ros::Time& time, const ros::Duration& period)
{
  odom2base_.header.stamp = time;
//  double yaw = velocityJointHandles_[2].getPosition();
  yaw_ += (wheelJointHandles_[0].getVelocity()-wheelJointHandles_[1].getVelocity()) *  (time.toSec() - lastTime_.toSec());
  tf2::Quaternion quaternion;
//  quaternion.setRPY(0, 0, yaw);
  quaternion.setRPY(0, 0, yaw_);
  odom2base_.transform.rotation.x = quaternion.x();
  odom2base_.transform.rotation.y = quaternion.y();
  odom2base_.transform.rotation.z = quaternion.z();
  odom2base_.transform.rotation.w = quaternion.w();

//  odom2base_.transform.translation.x = velocityJointHandles_[0].getPosition();
//  odom2base_.transform.translation.y = velocityJointHandles_[1].getPosition();
  double carXs = (wheelJointHandles_[0].getVelocity()+wheelJointHandles_[1].getVelocity()) / 2  *  (time.toSec() - lastTime_.toSec());
  odom2base_.transform.translation.x += carXs * sin(yaw_);
  odom2base_.transform.translation.y += carXs * cos(yaw_);
  odom2base_.transform.translation.z = 0;

  tfRtBroadcaster_.sendTransform(odom2base_);
}

void UprightController::updateStateEstimation(const ros::Time& time, const ros::Duration& period)
{
  //  Diablo odom info
  currentObservation_.state(0) = odom2base_.transform.translation.x;
  currentObservation_.state(1) = odom2base_.transform.translation.y;
  currentObservation_.state(2) = yaw_;

  if (updatePolicy_)
  {
    currentObservation_.state(9) = (wheelJointHandles_[0].getVelocity() + wheelJointHandles_[1].getVelocity())/2;
    currentObservation_.state(10) = wheelJointHandles_[0].getVelocity() - wheelJointHandles_[1].getVelocity();
    //  All accs are set to zero due to ros:: inaccurate and unstable time
    currentObservation_.state(17) = mpcMrtInterface_.get()->getPolicy().stateTrajectory_[1](18);
    currentObservation_.state(18) = mpcMrtInterface_.get()->getPolicy().stateTrajectory_[1](18);
  }
  else
  {
    currentObservation_.state(9) = 0;
    currentObservation_.state(10) =0.;
    currentObservation_.state(17) = 0;
    currentObservation_.state(18) =0.;
  }



  std_msgs::Float64MultiArray velOutMsg;
  //  arm info
  for (int i = 0; i < 6; ++i)
  {
      currentObservation_.state(3 + i) = hybridJointHandles_[i].getPosition();
//      velLPFs_[i].input(hybridJointHandles_[i].getVelocity());
//      currentObservation_.state(11 + i) = velLPFs_[i].output();
//      currentObservation_.state(11 + i) = hybridJointHandles_[i].getVelocity();
    //  All accs are set to zero due to ros:: inaccurate and unstable time
      velOutMsg.data.push_back(velLPFs_[i].output());
      double dt = ros::Time::now().toSec() - lastObservation_.time;
//      currentObservation_.state(19 + i) = (currentObservation_.state(11 + i) - lastObservation_.state(11 + i))/dt;
      if (updatePolicy_)
      {
        currentObservation_.state(3 + i) = mpcMrtInterface_.get()->getPolicy().stateTrajectory_[1](3 + i);
        currentObservation_.state(19 + i) = mpcMrtInterface_.get()->getPolicy().stateTrajectory_[1](19 + i);
      }
      else
      {
        currentObservation_.state(3 + i) = 0;
        currentObservation_.state(19 + i) =0.;
      }
  }
  velLPFPub_.publish(velOutMsg);
  lastObservation_ = currentObservation_;
  currentObservation_.time += period.toSec();
}

void UprightController::starting(const ros::Time& time)
{
  // Update time data
  TimeData time_data;
  time_data.time = time;
  time_data.uptime = ros::Time(0.0);
  time_data_.initRT(time_data);

  updateTfOdom(time, ros::Duration(0.001));
  updateStateEstimation(time, ros::Duration(0.001));
  currentObservation_.input.setZero(INPUT_DIM);
  currentObservation_.state = mobileManipulatorInterface_->getInitialState();
  ocs2::vector_t initDesiredState(7);

  initDesiredState.head(3) << init_x_, init_y_, init_z_;
  initDesiredState.tail(4) << Eigen::Quaternion<ocs2::scalar_t>(1, 0, 0, 0).coeffs();
  ocs2::TargetTrajectories targetTrajectories({ currentObservation_.time }, { initDesiredState },
                                              { currentObservation_.input });
  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(targetTrajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
  {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(mobileManipulatorInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");
  mpcRunning_ = true;
}

UprightController::~UprightController()
{
  controllerRunning_ = false;
  if (mpcThread_.joinable())
  {
    mpcThread_.join();
  }
  std::cerr << "###############################################################"
               "#########";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
}

void UprightController::setupMpc(ros::NodeHandle& nh)
{
  const std::string robotName = "diablo_manipulator";

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<ddt::RosReferenceManager>(
      "/mobile_manipulator", mobileManipulatorInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  // MPC
  mpc_ = mobileManipulatorInterface_->get_mpc();
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>("/mobile_manipulator_mpc_observation", 1);
}

void UprightController::setupMrt()
{
  mpcMrtInterface_ = std::make_shared<ocs2::MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&mobileManipulatorInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_)
    {
      try
      {
        ocs2::executeAndSleep(
            [&]() {
              if (mpcRunning_)
              {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            mobileManipulatorInterface_->mpcSettings().mpcDesiredFrequency_);
      }
      catch (const std::exception& e)
      {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  // TODO : Add config in task.info
  ocs2::setThreadPriority(50, mpcThread_);
}

void UprightController::uprightAvoid(const ros::Time& time, const ros::Duration& period)
{}

void UprightController::EESpaceLock(const ros::Time& time, const ros::Duration& period)
{}

void UprightController::normal(const ros::Time& time, const ros::Duration& period)
{
  static ocs2::vector_t lastOptimizedState(25);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();
  // Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput,
                                   plannedMode);

  currentObservation_.input = optimizedInput;
  int index = 0;
  for (auto joint : jointHandles_)
  {
    joint.setCommand(currentObservation_.input(index));
    index++;
  }

  lastOptimizedState = optimizedState;
}

void UprightController::upright(const ros::Time& time, const ros::Duration& period)
{
  static ocs2::vector_t lastOptimizedState(25);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();
  updatePolicy_ = true;
  // Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput,
                                   plannedMode);
  currentObservation_.input = optimizedInput;

  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(
      mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand(), mpcMrtInterface_->getPerformanceIndices());
  optimizedStateTrajectoryPub_.publish(mpcPolicyMsg);

  //  for diablo control
//  velocityJointHandles_[0].setCommand(optimizedState(9));
//  velocityJointHandles_[2].setCommand(optimizedState(10));
  wheelJointHandles_[0].setCommand(optimizedState(9) - optimizedState(10));
  wheelJointHandles_[1].setCommand(optimizedState(9) + optimizedState(10));
  ROS_INFO_STREAM(optimizedState(9));
//  velocityJointHandles_[0].setCommand(0);
//  velocityJointHandles_[2].setCommand(0);

  ocs2::PrimalSolution wholeSolution = mpcMrtInterface_->getPolicy();
  auto stateTrajectory = wholeSolution.stateTrajectory_;
  auto timeTrajectory = wholeSolution.timeTrajectory_;
  // Creat Spline
  std::map<int, std::vector<interpolation::CubicSpline::Node>> jointTrajectorys;
//  for (size_t i = 0; i < effortJointHandles_.size(); ++i)
//  {
//    int jointId = static_cast<int>(i);
//    std::vector<interpolation::CubicSpline::Node> trajectoryPoints;
//    for (size_t j = 0; j < stateTrajectory.size(); ++j)
//    {
//      if (3 + i >= stateTrajectory[j].size() || 11 + i >= stateTrajectory[j].size())
//      {
//        throw std::out_of_range("State trajectory index out of range.");
//      }
//      interpolation::CubicSpline::Node jointPoint{};
//      jointPoint.position = stateTrajectory[j](3 + i);
//      jointPoint.velocity = stateTrajectory[j](11 + i);
//      jointPoint.time = timeTrajectory[j];
//      trajectoryPoints.push_back(jointPoint);
//    }
  for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
  {
    int jointId = static_cast<int>(i);
    std::vector<interpolation::CubicSpline::Node> trajectoryPoints;
    for (size_t j = 0; j < stateTrajectory.size(); ++j)
    {
      if (3 + i >= stateTrajectory[j].size() || 11 + i >= stateTrajectory[j].size())
      {
        throw std::out_of_range("State trajectory index out of range.");
      }
      interpolation::CubicSpline::Node jointPoint{};
      jointPoint.position = stateTrajectory[j](3 + i);
      jointPoint.velocity = stateTrajectory[j](11 + i);
      jointPoint.time = timeTrajectory[j];
      trajectoryPoints.push_back(jointPoint);
    }
    jointTrajectorys[jointId] = trajectoryPoints;
  }

  splineTrajectory_ = std::make_shared<interpolation::MultiJointTrajectory>(jointTrajectorys);
  std_msgs::Float64MultiArray gravityMsg;
  for (int i = 0; i < 6; ++i)
  {
//    double posError = wholeSolution.stateTrajectory_[2](3 + i) - effortJointHandles_[i].getPosition();
//    double velError = optimizedState(11 + i) - effortJointHandles_[i].getVelocity();
//    double commanded_effort = pids_[i].computeCommand(posError, period);
//    auto desJointStates = splineTrajectory_->getStateAtTime(time_data_.readFromRT()->uptime.toSec() + 0.01)[i];
//    double commanded_effort =
//        pids_[i].computeCommand(desJointStates.position - effortJointHandles_[i].getPosition(), period);
    double gravityFF = endEffectorInterface_->getDynamics().G(i);
    gravityMsg.data.push_back(gravityFF);
//    effortJointHandles_[i].setCommand(commanded_effort + gravityFF);

    hybridJointHandles_[i].setCommand(wholeSolution.stateTrajectory_[8](3 + i), wholeSolution.stateTrajectory_[8](11 + i), pids_[i].getGains().p_gain_, pids_[i].getGains().d_gain_, gravityFF);
  }
  gravityCompPub_.publish(gravityMsg);
  lastOptimizedState = optimizedState;
}

ocs2_msgs::mpc_flattened_controller
UprightController::createMpcPolicyMsg(const ocs2::PrimalSolution& primalSolution, const ocs2::CommandData& commandData,
                                      const ocs2::PerformanceIndex& performanceIndices)
{
  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;

  mpcPolicyMsg.initObservation = ocs2::ros_msg_conversions::createObservationMsg(commandData.mpcInitObservation_);
  mpcPolicyMsg.planTargetTrajectories =
      ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(commandData.mpcTargetTrajectories_);
  mpcPolicyMsg.modeSchedule = ocs2::ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_);
  mpcPolicyMsg.performanceIndices =
      ocs2::ros_msg_conversions::createPerformanceIndicesMsg(commandData.mpcInitObservation_.time, performanceIndices);

  switch (primalSolution.controllerPtr_->getType())
  {
    case ocs2::ControllerType::FEEDFORWARD:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD;
      break;
    case ocs2::ControllerType::LINEAR:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR;
      break;
    default:
      throw std::runtime_error("MPC_ROS_Interface::createMpcPolicyMsg: Unknown ControllerType");
  }

  // maximum length of the message
  const size_t N = primalSolution.timeTrajectory_.size();

  mpcPolicyMsg.timeTrajectory.clear();
  mpcPolicyMsg.timeTrajectory.reserve(N);
  mpcPolicyMsg.stateTrajectory.clear();
  mpcPolicyMsg.stateTrajectory.reserve(N);
  mpcPolicyMsg.data.clear();
  mpcPolicyMsg.data.reserve(N);
  mpcPolicyMsg.postEventIndices.clear();
  mpcPolicyMsg.postEventIndices.reserve(primalSolution.postEventIndices_.size());

  // time
  for (auto t : primalSolution.timeTrajectory_)
  {
    mpcPolicyMsg.timeTrajectory.emplace_back(t);
  }

  // post-event indices
  for (auto ind : primalSolution.postEventIndices_)
  {
    mpcPolicyMsg.postEventIndices.emplace_back(static_cast<uint16_t>(ind));
  }

  // state
  for (size_t k = 0; k < N; k++)
  {
    ocs2_msgs::mpc_state mpcState;
    mpcState.value.resize(primalSolution.stateTrajectory_[k].rows());
    for (size_t j = 0; j < primalSolution.stateTrajectory_[k].rows(); j++)
    {
      mpcState.value[j] = primalSolution.stateTrajectory_[k](j);
    }
    mpcPolicyMsg.stateTrajectory.emplace_back(mpcState);
  }  // end of k loop

  // input
  for (size_t k = 0; k < N; k++)
  {
    ocs2_msgs::mpc_input mpcInput;
    mpcInput.value.resize(primalSolution.inputTrajectory_[k].rows());
    for (size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++)
    {
      mpcInput.value[j] = primalSolution.inputTrajectory_[k](j);
    }
    mpcPolicyMsg.inputTrajectory.emplace_back(mpcInput);
  }  // end of k loop

  // controller
  ocs2::scalar_array_t timeTrajectoryTruncated;
  std::vector<std::vector<float>*> policyMsgDataPointers;
  policyMsgDataPointers.reserve(N);
  for (auto t : primalSolution.timeTrajectory_)
  {
    mpcPolicyMsg.data.emplace_back(ocs2_msgs::controller_data());

    policyMsgDataPointers.push_back(&mpcPolicyMsg.data.back().data);
    timeTrajectoryTruncated.push_back(t);
  }  // end of k loop

  // serialize controller into data buffer
  primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

  return mpcPolicyMsg;
}

}  // namespace ddt
PLUGINLIB_EXPORT_CLASS(ddt::UprightController, controller_interface::ControllerBase)