#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
class Viewer
{
  private:
    ros::NodeHandle nh;
  public:
    Viewer() :nh(){
    	Mat image;
    	image=imread("mario.png");
    	ROS_INFO("1");
    	if(!image.data){
    		ROS_INFO("Can't open the image");
    	}
    	else{
	    	cvNamedWindow("Disp");
	    	imshow("Disp",image);
	    	ROS_INFO("2");
	    	cvWaitKey(0);
	    	cvDestroyWindow("Disp");
	    	ROS_INFO("Looped");
   		}
    }    
    void loop()
    {
      ros::Rate rate(1);
      while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
      }
    }
};
int main(int argc, char **argv)
{
  ros::init(argc,argv,"image_viewer");
  Viewer viewer;
  viewer.loop();
}
