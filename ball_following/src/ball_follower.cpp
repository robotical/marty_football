/**
 * @file      ball_follower.cpp
 * @brief     Marty follows the ball, and attempts to score
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2017-01-27
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <ball_following/ball_follower.hpp>

BallFollower::BallFollower(ros::NodeHandle& nh) : nh_(nh),
  tfListener_(tfBuffer_) {
  this->loadParams();
  this->init();
  this->rosSetup();
  ROS_INFO("BallFollower Ready!");
}

BallFollower::~BallFollower() {
  ros::param::del("ball_follower");
  command_srv_.call(stop_);
}

void BallFollower::loadParams() {
  nh_.param<int>("turn_amount", turn_amount_, 10);
  nh_.param<int>("ball_y_th", ball_y_th_, 90);
  nh_.param<int>("ball_x_th", ball_x_th_, 20);
  nh_.param<int>("ball_size_th", ball_size_th_, 30);
  nh_.param<int>("goal_th", goal_th_, 20);
  nh_.param<double>("apriltag_off", apriltag_off_, -7.5);
}

void BallFollower::init() {
  stop_.request.data.push_back(stop_.request.CMD_STOP);

  follow_.request.data.push_back(follow_.request.CMD_WALK);
  follow_.request.data.push_back(1);          //  Steps
  follow_.request.data.push_back(0);          //  Turn
  follow_.request.data.push_back(1800);       //  MoveTime
  follow_.request.data.push_back(40);         //  StepLength

  turn_.request.data.push_back(turn_.request.CMD_WALK);
  turn_.request.data.push_back(2);            //  Steps
  turn_.request.data.push_back(turn_amount_); //  Turn
  turn_.request.data.push_back(1800);         //  MoveTime
  turn_.request.data.push_back(0);            //  StepLength

  walk_.request.data.push_back(walk_.request.CMD_WALK);
  walk_.request.data.push_back(1);            //  Steps
  walk_.request.data.push_back(0);            //  Turn
  walk_.request.data.push_back(1800);         //  MoveTime
  walk_.request.data.push_back(0);            //  StepLength
  walk_.request.data.push_back(-1);           //  Side

  side_step_.request.data.push_back(side_step_.request.CMD_SIDESTEP);
  side_step_.request.data.push_back(0);      //  CMD_LEFT = 0
  side_step_.request.data.push_back(1);      //  Steps
  side_step_.request.data.push_back(1800);   //  MoveTime
  side_step_.request.data.push_back(50);     //  StepLength

  kick_.request.data.push_back(kick_.request.CMD_KICK);
  kick_.request.data.push_back(1);            //  CMD_LEFT = 0
  kick_.request.data.push_back(1800);         //  MoveTime
}

void BallFollower::rosSetup() {
  ball_pos_sub_ = nh_.subscribe("ball_pos", 1000,
                                &BallFollower::ballCB, this);
  command_srv_ = nh_.serviceClient<marty_msgs::Command>("command");
  action_timer_ = nh_.createTimer(ros::Duration(refresh_time_),
                                  &BallFollower::acCB, this);
}

void BallFollower::ballCB(const geometry_msgs::Pose2D& msg) {
  ball_pos_ = msg;
}

void BallFollower::followBall() {
  if (ball_pos_.x < 160 - ball_x_th_) {
    ROS_INFO("Walking Left");
    follow_.request.data[2] = turn_amount_;
  } else if (ball_pos_.x > 160 + ball_x_th_) {
    ROS_INFO("Walking Right");
    follow_.request.data[2] = -turn_amount_;
  } else {
    ROS_INFO("Walking Straight");
    follow_.request.data[2] = 0;
  }
  commands_.push(follow_);
}

void BallFollower::alignToGoal() {
  ROS_INFO("Aligning to Goal");
  if (ball_pos_.x < 160) {
    side_step_.request.data[1] = 0;         // Sidestep Left
  } else {
    side_step_.request.data[1] = 1;         // Sidestep Right
  }
  if (goal_angle_ > 0) {
    ROS_WARN("ALIGN LEFT");
    turn_.request.data[2] = turn_amount_;   // Turn Left
  } else {
    ROS_WARN("ALIGN RIGHT");
    turn_.request.data[2] = -turn_amount_;  // Turn Right
  }
  walk_.request.data[4] = 0;    // Recover after turn
  walk_.request.data[5] = -1;   // No side preference
  commands_.push(side_step_);
  commands_.push(turn_);
  commands_.push(walk_);
}

void BallFollower::turnAround() {
  ROS_INFO("Turning Around");
  turn_.request.data[2] = turn_amount_;
  commands_.push(turn_);
}

void BallFollower::kickBall() {
  ROS_INFO("Kicking Ball");
  walk_.request.data[5] = -1;  // No side preference
  walk_.request.data[4] = 50;
  commands_.push(walk_);
  commands_.push(walk_);
  if (ball_pos_.x < 160) {
    walk_.request.data[5] = 1;  // Step Right
    kick_.request.data[1] = 0;  // Kick Left
  } else {
    walk_.request.data[5] = 0;  // Step Left
    kick_.request.data[1] = 1;  // Kick Right
  }
  walk_.request.data[4] = 50;
  commands_.push(walk_);
  commands_.push(kick_);
}

void BallFollower::stop() {
  ROS_INFO("Stopping");
  commands_.push(stop_);
}

void BallFollower::acCB(const ros::TimerEvent& e) {
  this->updateGoalPos();
  // Ball Checks
  goal_angle_ = (atan2(goal_pos_.y, goal_pos_.x) / M_PI) * 180;
  if ((ball_pos_.x == 0) && (ball_pos_.y == 0))
  {ball_found_ = false;} else {ball_found_ = true;}
  if ((ball_pos_.x < 160 + ball_x_th_) &&
      (ball_pos_.x > 160 - ball_x_th_) )
  {ball_aligned_ = true;} else {ball_aligned_ = false;}
  if (std::fabs(goal_angle_) < goal_th_)
  {goal_aligned_ = true;} else {goal_aligned_ = false;}
  if ((ball_pos_.theta > ball_size_th_) && (ball_pos_.y > ball_y_th_))
  {ball_close_ = true;} else {ball_close_ = false;}
  ROS_INFO_STREAM("BallPos X: " << ball_pos_.x <<
                  " BallPos Y: " << ball_pos_.y <<
                  " BallSize: " << ball_pos_.theta <<
                  " GoalAngle: " << goal_angle_);
  ROS_INFO_STREAM("BallFound: " << ball_found_ <<
                  " BallAligned: " << ball_aligned_ <<
                  " GoalAligned: " << goal_aligned_ <<
                  " BallClose: " << ball_close_ );
  if (commands_.size() == 0) {
    // Choose new action
    if (!ball_found_) {
      this->turnAround();
    } else if (ball_found_ && ball_aligned_ && goal_aligned_ && ball_close_)  {
      this->kickBall();
    } else if (ball_found_ && ball_close_) {
      this->alignToGoal();
    } else if (ball_found_ && !ball_close_) {
      this->followBall();
    } else {
      ROS_WARN("None of the ball checks passed");
      this->stop();
    }
  }
  if (commands_.size() > 0) {
    // Execute last queued action
    ROS_INFO_STREAM(commands_.front().request);
    command_srv_.call(commands_.front());
    commands_.pop();
  }
}

void BallFollower::updateGoalPos() {
  geometry_msgs::TransformStamped tf;
  try {
    tf = tfBuffer_.lookupTransform("base_link", "goal_1", ros::Time(0));
    goal_pos_.x = tf.transform.translation.x;
    goal_pos_.y = tf.transform.translation.y + (apriltag_off_ / 100);
  } catch (tf2::TransformException& ex) {
    // ROS_WARN("%s", ex.what());
  }
}

void BallFollower::stopRobot() {
  command_srv_.call(stop_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ball_follower");
  ros::NodeHandle nh("");
  BallFollower ball_follower(nh);

  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ball_follower.stop();
  return 0;
}
