/**
 * @file ign_hw_interface.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief ign_hw_interface
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */
#pragma once

#include <boost/scoped_ptr.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

namespace ignition_hw_interface {

/**
 * @brief The IgnHardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class IgnHardwareInterface : public hardware_interface::RobotHW {
public:
  /**
   * @brief Construct a new Ignition Hardware Interface object
   * @param nh Root level ROS node handle
   */
  IgnHardwareInterface(const ros::NodeHandle &nh);
  /**
   * @brief Default Destructor for the Ignition Hardware Interface object
   */
  virtual ~IgnHardwareInterface() = default;
  /**
   * @brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * @returns True, if the setup was performed successfully
   *
   */
  virtual bool init();
  /**
   * @brief Read method of the control loop. Reads a messages from the robot and handles and
   * publishes the information as needed.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time &time, const ros::Duration &period) override;
  /**
   * @brief Write method of the control loop. Writes target joint positions to the robot to be read
   * by its PDL programs.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time &time, const ros::Duration &period) override;
  /**
   * @brief Keeps the connection alive by sending a standard message (NOT USED)
   *
   * @return true on successful write
   * @return false on failed write
   */
  bool holdConnection();
  /**
   * @brief Prints a double vector in the console for debugging
   *
   * @param vec The vector to be printed in the console
   */
  void printVector(const std::vector<double> &vec);
  /**
   * @brief Checks if the command is zero
   *
   * @param vec The vector to be checked if it is 0
   */
  bool ifZero(const std::vector<double> &vec);
  /**
   * @brief Assigns the source vector into the destination vector
   *
   * @param src The source vector
   * @param dest The destination vector
   */
  void copyVector(const std::vector<double> &src, std::vector<double> &dest);
  /**
   * @brief Checks if a reset of the controllers is necessary
   */
  bool shouldResetControllers();
  /*!
   * \brief Preparation to start and stop loaded controllers.
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   *
   * \returns True, if the controllers can be switched
   */
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                             const std::list<hardware_interface::ControllerInfo> &stop_list) override;
  /*!
   * \brief Starts and stops controllers.
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                        const std::list<hardware_interface::ControllerInfo> &stop_list) override;

  /*!
   * \brief Checks whether a resource list contains joints from this hardware interface
   *
   * True is returned as soon as one joint name from claimed_resources matches a joint from this
   * hardware interface.
   */
  bool checkControllerClaims(const std::set<std::string> &claimed_resources);
  /*!
   * \brief Checks if the program is running on the robot.
   *
   * \returns True, if the program is currently running, false otherwise.
   */
  bool isRobotProgramRunning() const;

protected:
  void ignJointStatesCB(const sensor_msgs::JointState &msg);

  std::string name_;             /**< Name of this class -> hardware_interface */
  ros::NodeHandle nh_, nh_priv_; /**< ROS NodeHandle objects required for parameters reading */
  // Configuration
  bool position_controller_running_;
  bool controllers_initialized_;
  bool robot_program_running_ = true; /* TODO */
  // Hardware Interfaces
  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  // States
  std::vector<double> joint_position_;
  sensor_msgs::JointState joint_states_;
  bool got_states_ = false;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  // Commands
  std::vector<double> joint_position_command_;
  // private
  size_t num_joints_;
  std::vector<std::string> joint_names_;
  uint64_t loop_hz_;
  bool packet_read_;
  std_msgs::Float64 joint1_cmd_, joint2_cmd_, joint3_cmd_, joint4_cmd_, joint5_cmd_, joint6_cmd_;
  // subscribers
  ros::Subscriber joint_state_sub_;
  // publishers
  ros::Publisher joint1_pub_, joint2_pub_, joint3_pub_, joint4_pub_, joint5_pub_, joint6_pub_;
};

} // namespace ignition_hw_interface
