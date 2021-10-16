/**
 * @file ign_hw_helpers.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief ign_hw_helpers
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#pragma once
#include <ignition_hw_interface/ign_hw_interface.h>
#include <sstream>

namespace ignition_hw_interface {

bool IgnHardwareInterface::shouldResetControllers() {
  return false;
}

bool IgnHardwareInterface::holdConnection() {
  // TODO add the implementation
  return true;
}

void IgnHardwareInterface::copyVector(const std::vector<double> &src, std::vector<double> &dest) {
  for (size_t i = 0; i < src.size(); i++)
    dest.at(i) = src.at(i);
}

bool IgnHardwareInterface::ifZero(const std::vector<double> &vec) {
  for (double val : vec)
    if (val != 0.0)
      return false;
  return true;
}

void IgnHardwareInterface::printVector(const std::vector<double> &vec) {
  ROS_INFO("Vector : %f %f %f %f %f %f", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
}

bool IgnHardwareInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                              const std::list<hardware_interface::ControllerInfo> &stop_list) {
  bool ret_val = true;
  if (controllers_initialized_ && !isRobotProgramRunning() && !start_list.empty()) {
    for (auto &controller : start_list) {
      if (!controller.claimed_resources.empty()) {
        ROS_ERROR_STREAM("Robot control is currently inactive. Starting controllers that claim resources is currently "
                         "not possible. Not starting controller '"
                         << controller.name << "'");
        ret_val = false;
      }
    }
  }

  controllers_initialized_ = true;
  return ret_val;
}

void IgnHardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                         const std::list<hardware_interface::ControllerInfo> &stop_list) {
  for (auto &controller_it : stop_list) {
    for (auto &resource_it : controller_it.claimed_resources) {
      if (checkControllerClaims(resource_it.resources)) {
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface") {
          position_controller_running_ = false;
        }
      }
    }
  }
  for (auto &controller_it : start_list) {
    for (auto &resource_it : controller_it.claimed_resources) {
      if (checkControllerClaims(resource_it.resources)) {
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface") {
          position_controller_running_ = true;
        }
      }
    }
  }
}

bool IgnHardwareInterface::checkControllerClaims(const std::set<std::string> &claimed_resources) {
  for (const std::string &it : joint_names_) {
    for (const std::string &jt : claimed_resources) {
      if (it == jt) {
        return true;
      }
    }
  }
  return false;
}

bool IgnHardwareInterface::isRobotProgramRunning() const {
  return robot_program_running_;
}

} // namespace ignition_hw_interface