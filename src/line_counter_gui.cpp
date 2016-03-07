#include <ros/ros.h>
//#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>
//#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <vector>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <iomanip>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

const std::string WindowName = "Line counter gui";

class Subscribe
{

	//for minimap:
  int minimapWidth = 10;
  int minimapHeight = 10;
  double minimapMeter = 40;
  int minimapOrigoX = 300;
  int minimapOrigoY = 300;

  cv::Mat minimap;

  geometry_msgs::PoseStamped counterPose;
  geometry_msgs::PoseStamped mavrosPose;

public:
	Subscribe()
	{
		sub_ = n_.subscribe("/line_counter/pose", 1, &Subscribe::callback, this);
    mavsub_ = n_.subscribe("/kalman_filter/pose", 1, &Subscribe::callback_mavros, this);
      cv::namedWindow(WindowName);
      //cv::imshow(WindowName, minimap);
	}

	~Subscribe()
{
		cv::destroyWindow(WindowName);
}

  void drawMinimap(){
    minimap = cv::Mat(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
      for(int i = 0; i<=minimapHeight; i++){
        cv::line(minimap, cv::Point2i(minimapOrigoX - minimapWidth/2*minimapMeter,minimapOrigoY + (int)((i-minimapHeight/2)*minimapMeter)),
                cv::Point2i(minimapOrigoX + minimapWidth/2*minimapMeter, minimapOrigoY + (int)((i-minimapHeight/2)*minimapMeter)),
                cv::Scalar(255, 255, 255));
      }
      for(int i = 0; i<=minimapWidth; i++){
        cv::line(minimap, cv::Point2i(minimapOrigoX +(int)((i-minimapHeight/2)*minimapMeter),minimapOrigoY -minimapHeight/2*minimapMeter),
                cv::Point2i(minimapOrigoX +(int)((i-minimapHeight/2)*minimapMeter),minimapOrigoY + minimapHeight/2*minimapMeter),
                cv::Scalar(255, 255, 255));
      }
      int stateypos = (int)(minimapOrigoY - counterPose.pose.position.x*minimapMeter);
      int statexpos = (int)(minimapOrigoX - counterPose.pose.position.y*minimapMeter);
      cv::circle(minimap, cv::Point2i(statexpos, stateypos), 5, cv::Scalar(255, 0, 255));
      geometry_msgs::Quaternion q = counterPose.pose.orientation;
      float theta = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)) + CV_PI/2;
      //std::cout<<theta<<std::endl;
      cv::line(minimap, cv::Point2i(statexpos, stateypos), cv::Point2i(statexpos + (int)(10*cos(theta)), stateypos + (int)(-10*sin(theta))), cv::Scalar(0, 255, 0));

      stateypos = (int)(minimapOrigoY - mavrosPose.pose.position.x*minimapMeter);
      statexpos = (int)(minimapOrigoX - mavrosPose.pose.position.y*minimapMeter);
      cv::circle(minimap, cv::Point2i(statexpos, stateypos), 5, cv::Scalar(0, 255, 0));
      q = mavrosPose.pose.orientation;
      theta = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)) + CV_PI/2;
      //std::cout<<theta<<std::endl;
      cv::line(minimap, cv::Point2i(statexpos, stateypos), cv::Point2i(statexpos + (int)(10*cos(theta)), stateypos + (int)(-10*sin(theta))), cv::Scalar(255, 0, 0));
      cv::imshow(WindowName, minimap);
      cv::waitKey(2);
  }

	void callback(const geometry_msgs::PoseStamped& input)
	{
    counterPose = input;
    //drawMinimap();
	}

  void callback_mavros(const geometry_msgs::PoseStamped &pose){
    //return;
    mavrosPose = pose;
    drawMinimap();
  }

private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
  ros::Subscriber mavsub_;

};//End of class Subscribe

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_counter_gui");

	//Create an object of class Subscribe that will take care of everything
	Subscribe SAPObject;
	ros::spin();
	return 0;
}
