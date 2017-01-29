/**
 * @file      ball_follower.hpp
 * @brief     Marty follows the ball, and attempts to score
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2017-01-27
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#ifndef BALL_FOLLOWER_HPP
#define BALL_FOLLOWER_HPP

// System

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// Messages
#include <geometry_msgs/Pose2D.h>
#include <marty_msgs/Command.h>

// #include "opencv2/core/version.hpp"

class BallFollower {
 protected:
  ros::NodeHandle nh_;

  void loadParams();
  void init();
  void rosSetup();

 public:
  BallFollower(ros::NodeHandle& nh);
  ~BallFollower();

  void follow();

 private:
  void ballCB(const geometry_msgs::Pose2D& msg);

  // Flags
  bool ball_found_;

  // Parameters
  int ball_y_thresh_;
  int ball_x_thresh_;
  int turn_amount_;

  // Variables

  // ROS
  ros::Subscriber ball_pos_sub_;
  ros::ServiceClient move_srv_;
  geometry_msgs::Pose2D ball_pos_;
  marty_msgs::Command stop_;
};

#endif  /* BALL_FOLLOWER_HPP */
