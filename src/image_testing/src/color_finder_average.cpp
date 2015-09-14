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
//yellow balloon 160/190/140/170/30/60
/* DEFAULT VALUES */
const int blueLowerBound = 30;
const int blueUpperBound = 60;
const int greenLowerBound = 140;
const int greenUpperBound = 170;
const int redLowerBound = 160;
const int redUpperBound = 190;

Scalar colorScalar_BLUE = Scalar(255,255,0);
Scalar colorScalar_GREEN = Scalar(0,255,0);
Scalar colorScalar_RED = Scalar(0,0,255);

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
    image_sub = it.subscribe("/camera/image_raw", 1, 
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
    int pointx = 0;
    int pointy = 0;
    int i=0;
    int j=0;
    int columns = cv_ptr->image.cols;
    int rows = cv_ptr->image.rows;
    int centPointx = columns/2;
    int centPointy = rows/2;

    int blueLower;
    int blueUpper;
    int greenLower;
    int greenUpper;
    int redLower;
    int redUpper;

    nh.getParam("/colorranges/blueLowerBound",blueLower);
    nh.getParam("/colorranges/blueUpperBound",blueUpper);
    nh.getParam("/colorranges/greenLowerBound",greenLower);
    nh.getParam("/colorranges/greenUpperBound",greenUpper);
    nh.getParam("/colorranges/redLowerBound",redLower);
    nh.getParam("/colorranges/redUpperBound",redUpper);

    if(blueLower > blueUpper ||
      greenLower > greenUpper ||
      redLower > redUpper){
      ROS_INFO("Color Range failure, lower bound > upper bound");
      ros::shutdown();
    }

    circle(cv_ptr->image,Point(columns/2,rows/2),2,colorScalar_BLUE,1);
    int outBlue = cv_ptr->image.at<Vec3b>(Point(centPointx,centPointy))[0];
    int outGreen = cv_ptr->image.at<Vec3b>(Point(centPointx,centPointy))[1];
    int outRed = cv_ptr->image.at<Vec3b>(Point(centPointx,centPointy))[2];
    ROS_INFO("B: %i G: %i R: %i",outBlue,outGreen,outRed);
    //BGR
    for(double cols = cv_ptr->image.cols; pointx < cols; pointx+=cols/accuracy){
      pointy=0;
      for(double rows = cv_ptr->image.rows; pointy < rows; pointy+=rows/accuracy){
        if(cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[0] > blueLower  &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[0] < blueUpper &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[1] > greenLower  &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[1] < greenUpper &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[2] > redLower &&
           cv_ptr->image.at<Vec3b>(Point(pointx,pointy))[2] < redUpper){
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
    int xTotal=0;
    int yTotal=0;
    int listSize = listX.size();

    if(listSize > 0){
      while(listX.size()>0){
        currX=listX.back();
        currY=listY.back();

        xTotal += currX;
        yTotal += currY;


        listX.pop_back();
        listY.pop_back();
        i++;
      }
      int xCentPt = xTotal / listSize;
      int yCentPt = yTotal / listSize;
      circle(cv_ptr->image,Point(xCentPt,yCentPt),3,colorScalar_GREEN,1);
      circle(cv_ptr->image,Point(xCentPt,yCentPt),2,colorScalar_RED,1);
      line(cv_ptr->image,Point(columns/2,rows/2-50),Point(xCentPt,yCentPt),colorScalar_RED,2);
      putText(cv_ptr->image,"Target Located",Point(columns/2+10,rows/2-60), 2, 0.7, colorScalar_RED,2);
    } 
    else{
      putText(cv_ptr->image,"Locating Target...",Point(columns/2+10,rows/2-60), 2, 0.7, colorScalar_RED,2);

    }
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
