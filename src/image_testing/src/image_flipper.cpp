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

//good set for medium blue: 100/140/80/110/35/55
//good set for bright orange: 0/25/35/50/110/125

/* DEFAULT VALUES */
const int blueLowerBound = 100;
const int blueUpperBound = 140;
const int greenLowerBound = 80;
const int greenUpperBound = 110;
const int redLowerBound = 35;
const int redUpperBound = 55;

Scalar colorScalar_BLUE = Scalar(255,255,0);
Scalar colorScalar_GREEN = Scalar(0,255,0);
Scalar colorScalar_RED = Scalar(255,0,0);

Scalar colorScalar = Scalar(0,255,0);

vector<int> listX;
vector<int> listY;

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
    image_pub = it.advertise("/camera/output_video", 1);

    nh.setParam("/colorranges/blueLowerBound", blueLowerBound);
    nh.setParam("/colorranges/blueUpperBound", blueUpperBound);
    nh.setParam("/colorranges/greenLowerBound", greenLowerBound);
    nh.setParam("/colorranges/greenUpperBound", greenUpperBound);
    nh.setParam("/colorranges/redLowerBound", redLowerBound);
    nh.setParam("/colorranges/redUpperBound", redUpperBound);

  }

  ~ImageConverter()
  {}
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //flip code 0 corresponds to vertical "flippage"
    flip(cv_ptr->image,cv_ptr->image,0);

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
