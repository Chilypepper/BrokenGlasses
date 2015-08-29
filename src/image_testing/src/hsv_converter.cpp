#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

const int testPointX=200;
const int testPointY=100;

const int accuracy = 200;
Scalar colorScalar_BLUE = Scalar(255,255,0);
Scalar colorScalar = Scalar(0,255,0);

vector<int> listX;
vector<int> listY;

Mat mask;

class ImageConverter
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  
public:
  ImageConverter()
    : it(nh)
  {
    image_sub = it.subscribe("/camera/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub = it.advertise("/camera/output_video_hsv", 1);
  }

  ~ImageConverter()
  {}
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_reds;
    vector<Mat> channels;

    IplImage *Rimg;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    split(cv_ptr->image,channels);
    Mat result_red(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
    Mat in3[] = { empty_channel, empty_channel, spl[2]};
    


    //Rimg=cvCreateImage(cvSize(cv_ptr->image.cols,cv_ptr->image.rows),
    //  IPL_DEPTH_8U,3);
    //BGR
    cvtColor(cv_ptr->image,cv_ptr->image,COLOR_BGR2HSV);

    inRange(cv_ptr->image,Scalar(0,50,40), Scalar(10,255,255), mask);

    image_pub.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;

}