/**
 * @file      ball_follower.cpp
 * @brief     Marty Core header providing access to Marty methods
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <ball_following/ball_follower.hpp>

BallFollower::BallFollower(ros::NodeHandle& nh) : nh_(nh) {
  this->loadParams();
  this->init();
  this->rosSetup();
  ROS_INFO("BallFollower Ready!");
}

BallFollower::~BallFollower() {
}

void BallFollower::loadParams() {
  nh_.param<int>("turn_amount", turn_amount_, 10);
  nh_.param<int>("ball_y_thresh", ball_y_thresh_, 90);
  nh_.param<int>("ball_x_thresh", ball_x_thresh_, 20);
}

void BallFollower::init() {
  ball_found_ = false;
  stop_.request.data.push_back(15);     // CMD_STOP = 15
}

void BallFollower::rosSetup() {
  ball_pos_sub_ = nh_.subscribe("/ball_pos", 1000, &BallFollower::ballCB, this);
  move_srv_ = nh_.serviceClient<marty_msgs::Command>("/command");
}

void BallFollower::ballCB(const geometry_msgs::Pose2D& msg) {
  ball_pos_ = msg;
}

void BallFollower::follow() {
  marty_msgs::Command move;
  ROS_INFO_STREAM("BALL POS X: " << ball_pos_.x << " Y: " << ball_pos_.y);
  move.request.data.push_back(3);     //  CMD_WALK = 3
  move.request.data.push_back(1);     //  Steps = 1
  move.request.data.push_back(0);     //  Turn = 0
  move.request.data.push_back(2000);  //  MoveTime = 1second
  if ((ball_pos_.x != 0) && (ball_pos_.y != 0)) {
    ball_found_ = true;
    if (ball_pos_.y > ball_y_thresh_) {
      if (ball_pos_.x < 160 - ball_x_thresh_) {
        ROS_INFO("Walking Left");
        move.request.data[2] = turn_amount_;
      } else if (ball_pos_.x > 160 + ball_x_thresh_) {
        ROS_INFO("Walking Right");
        move.request.data[2] = -turn_amount_;
      } else {
        ROS_INFO("Walking Straight");
      }
      move_srv_.call(move);
    } else { ROS_INFO("Ball too far!"); move_srv_.call(stop_); }
  } else { ROS_INFO("Ball not found..."); move_srv_.call(stop_); }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ball_follower");
  ros::NodeHandle nh("~");
  BallFollower ball_follower(nh);

  int i = 0;  //  Number of iterations
  int n = 20; //  Iteration when ball is followed

  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    if (i > n) {
      ball_follower.follow();
      i = 0;
    }
    i++;
    r.sleep();
  }
  return 0;
}
