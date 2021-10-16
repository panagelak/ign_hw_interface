/**
 * @file test_pub.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief test_pub
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float64.h>

class TestIgnJointPub {
public:
  TestIgnJointPub(ros::NodeHandle nh) : nh_(nh) {
    joint1_pub_ = nh_.advertise<std_msgs::Float64>("cmd_shoulder_pan_joint", 10);
    joint2_pub_ = nh_.advertise<std_msgs::Float64>("cmd_shoulder_lift_joint", 10);
    joint3_pub_ = nh_.advertise<std_msgs::Float64>("cmd_elbow_joint", 10);
    joint4_pub_ = nh_.advertise<std_msgs::Float64>("cmd_wrist_1_joint", 10);
    joint5_pub_ = nh_.advertise<std_msgs::Float64>("cmd_wrist_2_joint", 10);
    joint6_pub_ = nh_.advertise<std_msgs::Float64>("cmd_wrist_3_joint", 10);
    joint1_msg_.data = 0.;
    joint2_msg_.data = 0.;
    joint3_msg_.data = 0.;
    joint4_msg_.data = 0.;
    joint5_msg_.data = 0.;
    joint6_msg_.data = 0.;
  }
  void startPos() {
    joint1_msg_.data = 0.;
    joint2_msg_.data = 0.;
    joint3_msg_.data = 0.;
    joint4_msg_.data = 0.;
    joint5_msg_.data = 0.;
    joint6_msg_.data = 0.;
    joint1_pub_.publish(joint1_msg_);
    joint2_pub_.publish(joint2_msg_);
    joint3_pub_.publish(joint3_msg_);
    joint4_pub_.publish(joint4_msg_);
    joint5_pub_.publish(joint5_msg_);
    joint6_pub_.publish(joint6_msg_);
    ros::Duration(5.).sleep();
  }

  void sinPub() {
    // config
    int cycles = 2;
    double freq_1 = 0.25, freq_2 = 0.25, freq_3 = 0.25, freq_4 = 0.25, freq_5 = 0.25,
           freq_6 = 0.25; // 4 seconds for 1 cycle
    double ampl_1 = 1.57, ampl_2 = 1.57, ampl_3 = 1.57, ampl_4 = 1.57, ampl_5 = 1.57, ampl_6 = 1.57;
    double start_pos_1 = 0.0, start_pos_2 = 0.0, start_pos_3 = 0.0, start_pos_4 = 0.0, start_pos_5 = 0.0,
           start_pos_6 = 0.0;
    bool en_1 = true, en_2 = false, en_3 = false, en_4 = false, en_5 = false, en_6 = false;

    double pub_rate = 50.;

    for (int i = 0; i < 4 * cycles * pub_rate; i++) {
      if (en_1) {
        joint1_msg_.data = start_pos_1 + sin(2. * M_PI * freq_1 * (i * (1. / pub_rate))) * ampl_1;
        joint1_pub_.publish(joint1_msg_);
      }
      if (en_2) {
        joint2_msg_.data = start_pos_2 + sin(2. * M_PI * freq_2 * (i * (1. / pub_rate))) * ampl_2;
        joint2_pub_.publish(joint2_msg_);
      }
      if (en_3) {
        joint3_msg_.data = start_pos_3 + sin(2. * M_PI * freq_3 * (i * (1. / pub_rate))) * ampl_3;
        joint3_pub_.publish(joint3_msg_);
      }
      if (en_4) {
        joint4_msg_.data = start_pos_4 + sin(2. * M_PI * freq_4 * (i * (1. / pub_rate))) * ampl_4;
        joint4_pub_.publish(joint4_msg_);
      }
      if (en_5) {
        joint5_msg_.data = start_pos_5 + sin(2. * M_PI * freq_5 * (i * (1. / pub_rate))) * ampl_5;
        joint5_pub_.publish(joint5_msg_);
      }
      if (en_6) {
        joint6_msg_.data = start_pos_6 + sin(2. * M_PI * freq_6 * (i * (1. / pub_rate))) * ampl_6;
        joint6_pub_.publish(joint6_msg_);
      }

      ros::Duration(1. / pub_rate).sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher joint1_pub_, joint2_pub_, joint3_pub_, joint4_pub_, joint5_pub_, joint6_pub_;
  std_msgs::Float64 joint1_msg_, joint2_msg_, joint3_msg_, joint4_msg_, joint5_msg_, joint6_msg_;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_ign_joint_pub");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  TestIgnJointPub handler(nh);

  handler.startPos();
  handler.sinPub();

  ros::waitForShutdown();

  return 0;
}