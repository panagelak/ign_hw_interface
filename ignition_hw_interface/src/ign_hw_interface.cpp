/**
 * @file ign_hw_interface.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief ign_hw_interface
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include <sstream>

#include <ignition_hw_interface/ign_hw_helpers.h>
#include <ignition_hw_interface/ign_hw_interface.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace ignition_hw_interface {

IgnHardwareInterface::IgnHardwareInterface(const ros::NodeHandle &nh)
    : name_("ignition_hw_interface"), nh_(nh), nh_priv_(nh, name_), position_controller_running_(false),
      controllers_initialized_(false) {}

bool IgnHardwareInterface::init() {
  // Get Ignition Hardware Interface parameters
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_priv_, "joints", joint_names_);
  rosparam_shortcuts::shutdownIfError(name_, error);
  // Resize vectors
  num_joints_ = joint_names_.size();
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);

  // Create ros_control interfaces
  for (size_t i = 0; i < num_joints_; ++i) {
    ROS_DEBUG_STREAM("[ign_hw_interface] Registering handles for joint " << joint_names_[i]);
    try {
      // Create joint state interface
      js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                        &joint_velocity_[i], &joint_effort_[i]));
      // Create position joint interface
      pj_interface_.registerHandle(
          hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM("[ign_hw_interface] " << e.what());
      return false;
    }
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
  // subscribers
  joint_state_sub_ = nh_.subscribe("joint_states_ign", 10, &IgnHardwareInterface::ignJointStatesCB, this);
  // publishers
  joint1_pub_ = nh_.advertise<std_msgs::Float64>("cmd_shoulder_pan_joint", 10);
  joint2_pub_ = nh_.advertise<std_msgs::Float64>("cmd_shoulder_lift_joint", 10);
  joint3_pub_ = nh_.advertise<std_msgs::Float64>("cmd_elbow_joint", 10);
  joint4_pub_ = nh_.advertise<std_msgs::Float64>("cmd_wrist_1_joint", 10);
  joint5_pub_ = nh_.advertise<std_msgs::Float64>("cmd_wrist_2_joint", 10);
  joint6_pub_ = nh_.advertise<std_msgs::Float64>("cmd_wrist_3_joint", 10);

  return true;
}

void IgnHardwareInterface::ignJointStatesCB(const sensor_msgs::JointState &msg) {
  // joint_states_ = msg;
  if (position_controller_running_) {
    joint_position_[0] = msg.position[0];
    joint_position_[1] = msg.position[1];
    joint_position_[2] = msg.position[2];
    joint_position_[3] = msg.position[3];
    joint_position_[4] = msg.position[4];
    joint_position_[5] = msg.position[5];
    got_states_ = true;
  }
}

void IgnHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
  // dummy feedback assume perfect execution
  if (!(position_controller_running_ || got_states_ == false))
    copyVector(joint_position_command_, joint_position_);
  return;
}

void IgnHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
  joint1_cmd_.data = joint_position_command_[0];
  joint2_cmd_.data = joint_position_command_[1];
  joint3_cmd_.data = joint_position_command_[2];
  joint4_cmd_.data = joint_position_command_[3];
  joint5_cmd_.data = joint_position_command_[4];
  joint6_cmd_.data = joint_position_command_[5];
  joint1_pub_.publish(joint1_cmd_);
  joint2_pub_.publish(joint2_cmd_);
  joint3_pub_.publish(joint3_cmd_);
  joint4_pub_.publish(joint4_cmd_);
  joint5_pub_.publish(joint5_cmd_);
  joint6_pub_.publish(joint6_cmd_);
}

} // namespace ignition_hw_interface
