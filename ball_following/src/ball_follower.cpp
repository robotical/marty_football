#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Pose2D.h>
#include <marty_msgs/Command.h>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
#define VERS 0
#elif CV_MAJOR_VERSION == 3
#define VERS 1
#endif

class BallFollower {
  ros::NodeHandle nh_;

  ros::Subscriber ball_pos_sub_;
  ros::ServiceClient move_srv_;
  geometry_msgs::Pose2D ball_pos_;
  bool ball_found_;

 public:
  BallFollower() {

    ball_pos_sub_ = nh_.subscribe("/ball_pos", 1000, &BallFollower::ballCB, this);
    move_srv_ = nh_.serviceClient<marty_msgs::Command>("/command");
    ball_found_ = false;

  }

  ~BallFollower() {
    marty_msgs::Command move;
    move.request.data.push_back(15);
    move_srv_.call(move);
  }

  void ballCB(const geometry_msgs::Pose2D& msg) {
    ball_pos_ = msg;
  }

  void follow() {
    marty_msgs::Command move;
    ROS_INFO_STREAM("BALL POS X: " << ball_pos_.x << " Y: " << ball_pos_.y);
    move.request.data.push_back(3); //  CMD_WALK
    move.request.data.push_back(1);                             //  Steps = 1
    move.request.data.push_back(0);                             //  Turn = 0
    move.request.data.push_back(
      2000);                          //  MoveTime = 1second
    int turn_amount = 10;
    if ((ball_pos_.x != 0) && (ball_pos_.y != 0))
      if (ball_pos_.y > 90) {
        if (ball_pos_.x < 140) {
          ROS_INFO("LEFT");
          move.request.data[2] = turn_amount;
        } else if (ball_pos_.x > 180) {
          ROS_INFO("RIGHT");
          move.request.data[2] = -turn_amount;
        } else {
          ROS_INFO("STRAIGHT");
        }
        move_srv_.call(move);
      }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ball_follower");
  BallFollower ball_follower;
  ros::Rate r(10);
  int i = 0;
  int n = 20;

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
