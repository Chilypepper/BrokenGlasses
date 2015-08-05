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
    image_pub = it.advertise("/image_converter/output_video", 1);
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
    int pointx = 0;
    int pointy = 0;
    int i=0;
    int j=0;
    int columns = cv_ptr->image.cols;
    int rows = cv_ptr->image.rows;

    circle(cv_ptr->image,Point(columns/2,rows/2),2,colorScalar_BLUE,1);
    int outRed = cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[0];
    int outGreen = cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[1];
    int outBlue = cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[2];
    ROS_INFO("R: %i G: %i B: %i",outRed,outGreen,outBlue);
    //BGR
    for(double cols = cv_ptr->image.cols; pointx < cols; pointx+=cols/accuracy){
      pointy=0;
      for(double rows = cv_ptr->image.rows; pointy < rows; pointy+=rows/accuracy){
        if(cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[0] > 0  &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[0] < 15 &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[1] > 20  &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[1] < 70 &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[2] > 40  &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[2] < 15){
            #ifdef findPoint
              circle(cv_ptr->image,Point(pointx,pointy),0,colorScalar_BLUE,1);
            #endif
          listX.push_back(pointx);
          listY.push_back(pointy);
          j++;
        }
      }
    }
    int largestX = 0;
    int largestY = 0;
    int smallestX = cv_ptr->image.cols;
    int smallestY = cv_ptr->image.rows;

    int currX=0;
    int currY=0; //#india
    while(listX.size()>0){
      currX=listX.back();
      currY=listY.back();
      if(currX > largestX){
        largestX=currX;
      }
      if(currY > largestY){
        largestY=currY;
      }
      if(currX < smallestX){
        smallestX = currX;
      }
      if(currY < smallestY){
        smallestY = currY;
      }
      listX.pop_back();
      listY.pop_back();
      i++;
    }
    
    rectangle(cv_ptr->image,Point(largestX,largestY),Point(smallestX,smallestY),colorScalar_BLUE,5,8,0);
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
