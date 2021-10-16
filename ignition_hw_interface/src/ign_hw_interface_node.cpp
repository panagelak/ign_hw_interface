/**
 * @file hw_interface_node.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief hw_interface_node
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include <ros/ros.h>

#include <csignal>
#include <ignition_hw_interface/hw_control_loop.h>
#include <ignition_hw_interface/ign_hw_interface.h>

boost::shared_ptr<ignition_hw_interface::IgnHardwareInterface> ign_hw_interface_ptr;
boost::shared_ptr<hw_control_loop::HWControlLoop> ign_hw_control_loop_ptr;

void signalHandler(int signum) {

  ROS_WARN_STREAM("[ignition_hw_interface] Interrupt signal (" << signum << ") received.\n");
  ign_hw_control_loop_ptr.reset();
  ign_hw_interface_ptr.reset();
  exit(signum);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ign_hardware_interface");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);
  // Create the hardware interface
  ign_hw_interface_ptr.reset(new ignition_hw_interface::IgnHardwareInterface(nh));
  if (!ign_hw_interface_ptr->init()) {
    ROS_ERROR_STREAM("[ignition_hw_interface] Could not correctly initialize robot. Exiting");
    exit(1);
  }
  ROS_INFO_STREAM("[ignition_hw_interface] HW interface initialized");
  // Start the control loop
  ign_hw_control_loop_ptr.reset(new hw_control_loop::HWControlLoop(nh, ign_hw_interface_ptr));
  ign_hw_control_loop_ptr->run(); // Blocks until shutdown signal received

  return 0;
}
