#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Pose2D.h>
#include <camera_test/dyn_paramConfig.h>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
#define VERS 0
#elif CV_MAJOR_VERSION == 3
#define VERS 1
#endif

class BallTracker {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher HueComp_pub_;
  image_transport::Publisher SatComp_pub_;
  image_transport::Publisher ValComp_pub_;
  image_transport::Publisher Detection_pub_;

  dynamic_reconfigure::Server<camera_test::dyn_paramConfig> server;
  dynamic_reconfigure::Server<camera_test::dyn_paramConfig>::CallbackType f;

  ros::Publisher ball_pos_pub_;
  cv::Mat frame;
  sensor_msgs::ImagePtr new_msg;
  sensor_msgs::ImagePtr hue_msg;
  sensor_msgs::ImagePtr sat_msg;
  sensor_msgs::ImagePtr val_msg;
  sensor_msgs::ImagePtr det_msg;
  cv_bridge::CvImagePtr cv_ptr;

  int hmn = 124;
  int hmx = 178;
  int smn = 60;
  int smx = 224;
  int vmn = 55;
  int vmx = 255;

 public:
  void callback(camera_test::dyn_paramConfig& config, uint32_t level) {
    hmn = config.hmin;
    hmx = config.hmax;
    smn = config.smin;
    smx = config.smax;
    vmn = config.vmin;
    vmx = config.vmax;
  }
  BallTracker()
    : it_(nh_) {
    if (VERS == 0) {
      ROS_DEBUG("CV2");
    } else if (VERS == 1) {
      ROS_DEBUG("CV3");
    }
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
                               &BallTracker::imageCb, this);
    image_pub_ = it_.advertise("/camera/output_image", 1);
    HueComp_pub_ = it_.advertise("/camera/hue", 1);
    SatComp_pub_ = it_.advertise("/camera/sat", 1);
    ValComp_pub_ = it_.advertise("/camera/val", 1);
    Detection_pub_ = it_.advertise("/camera/detection", 1);

    ball_pos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/ball_pos", 1000);

    f = boost::bind(&BallTracker::callback, this, _1, _2);
    server.setCallback(f);

    // cv::namedWindow("HueComp");
    // cv::namedWindow("SatComp");
    // cv::namedWindow("ValComp");
    // cv::namedWindow("Detection");
    // cv::namedWindow("Tracking");

    // int hmin = 124;
    // cv::createTrackbar("hmin", "HueComp", &hmin, 179);
    // int hmax = 178;
    // cv::createTrackbar("hmax", "HueComp", &hmax, 179);

    // int smin = 60;
    // cv::createTrackbar("smin", "SatComp", &smin, 255);
    // int smax = 224;
    // cv::createTrackbar("smax", "SatComp", &smax, 255);

    // int vmin = 55;
    // cv::createTrackbar("vmin", "ValComp", &vmin, 255);
    // int vmax = 255;
    // cv::createTrackbar("vmax", "ValComp", &vmax, 255);
    ROS_INFO("BallTracker Ready!");
  }

  ~BallTracker() {
    cv::destroyAllWindows();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    try {
      // Transform ROS image into OpenCV image
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      frame = cv_ptr->image;
      // if (frame.empty()) {
      //   ROS_ERROR("FRAME EMPTY");
      // }
      // if (frame.data != NULL) {
      //   ROS_INFO("DATA NOT NULL");
      // }
      cv::Mat HSVImage;
      // Transform the colors into HSV
      cv::cvtColor(frame, HSVImage, CV_BGR2HSV);

      // Thresholds
      // auto hmn = cv::getTrackbarPos("hmin", "HueComp");
      // auto hmx = cv::getTrackbarPos("hmax", "HueComp");
      // auto smn = cv::getTrackbarPos("smin", "SatComp");
      // auto smx = cv::getTrackbarPos("smax", "SatComp");
      // auto vmn = cv::getTrackbarPos("vmin", "ValComp");
      // auto vmx = cv::getTrackbarPos("vmax", "ValComp");

      cv::Mat hthresh;
      cv::Mat sthresh;
      cv::Mat vthresh;
      cv::inRange(HSVImage, cv::Scalar(hmn, 0, 0), cv::Scalar(hmx, 255, 255),
                  hthresh);
      cv::inRange(HSVImage, cv::Scalar(0, smn, 0), cv::Scalar(255, smx, 255),
                  sthresh);
      cv::inRange(HSVImage, cv::Scalar(0, 0, vmn), cv::Scalar(255, 255, vmx),
                  vthresh);

      cv::Mat and1, tracking;
      cv::bitwise_and(hthresh, sthresh, and1);
      cv::bitwise_and(vthresh, and1, tracking);

      cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
      cv::Mat dilation, closing, closing2;

      cv::dilate(tracking, dilation, kernel);
      cv::morphologyEx(dilation, closing, cv::MORPH_CLOSE, kernel);
      cv::GaussianBlur(closing, closing2, cv::Size(5, 5), 0);

      cv::Mat circles;
      cv::HoughCircles(closing2, circles, CV_HOUGH_GRADIENT, 2, 120, 120, 10, 15, 0);


      geometry_msgs::Pose2D ball_msg;
      if (circles.rows > 0) {
        for (int i = 0; i < circles.rows; ++i) {
          ball_msg.x = round(circles.at<float>(0, 0));
          ball_msg.y = round(circles.at<float>(0, 1));
          ball_msg.theta = round(circles.at<float>(0, 2));
          cv::circle(frame, cv::Point(ball_msg.x, ball_msg.y), ball_msg.theta,
                     CV_RGB(255, 0, 0));
        }
      }
      new_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      image_pub_.publish(new_msg);
      hue_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", hthresh).toImageMsg();
      HueComp_pub_.publish(hue_msg);
      sat_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", sthresh).toImageMsg();
      SatComp_pub_.publish(sat_msg);
      val_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", vthresh).toImageMsg();
      ValComp_pub_.publish(val_msg);
      det_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",
                                   closing2).toImageMsg();
      Detection_pub_.publish(det_msg);

      ball_pos_pub_.publish(ball_msg);


      // Update GUI Window
      // cv::imshow("HueComp", hthresh);
      // cv::imshow("SatComp", sthresh);
      // cv::imshow("ValComp", vthresh);
      // cv::imshow("Detection", closing2);
      // cv::imshow("Tracking", frame);
      // cv::waitKey(3);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ball_tracker");
  BallTracker ic;
  ros::spin();
  return 0;
}
