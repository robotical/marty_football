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
#include <math.h>
#include <queue>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>

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

  void followBall();
  void alignToGoal();
  void turnAround();
  void kickBall();
  void stop();
  void stopRobot();

 private:
  void ballCB(const geometry_msgs::Pose2D& msg);
  void acCB(const ros::TimerEvent& e);
  void updateGoalPos();

  // Flags
  bool ball_found_ = false;
  bool ball_aligned_ = false;
  bool goal_aligned_ = false;
  bool ball_close_ = false;

  // Parameters
  int ball_y_th_;
  int ball_x_th_;
  int turn_amount_;
  int ball_size_th_;
  int goal_th_;
  double apriltag_off_;
  double move_time_;
  double refresh_time_;
  int last_seen_dir_;

  // Variables
  float goal_angle_;
  std::queue<marty_msgs::Command> commands_;

  // ROS
  ros::Subscriber ball_pos_sub_;
  ros::ServiceClient command_srv_;
  geometry_msgs::Pose2D ball_pos_;  //  x,y Pixel coord, theta is ball size
  geometry_msgs::Pose2D goal_pos_;
  marty_msgs::Command stop_;
  marty_msgs::Command follow_;
  marty_msgs::Command turn_;
  marty_msgs::Command walk_;
  marty_msgs::Command side_step_;
  marty_msgs::Command kick_;

  // TF
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  ros::Timer action_timer_;
};

#endif  /* BALL_FOLLOWER_HPP */
