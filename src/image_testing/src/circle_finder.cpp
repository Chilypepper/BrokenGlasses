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

int edgeCount= 0;
//BGR
Scalar colorScalar_RED = Scalar(0,0,255);
Scalar colorScalar = Scalar(0,255,0);

vector<Point> pointList;
vector<Point> pointList2;

bool foundEdge=false;

int colorDifference(cv_bridge::CvImagePtr cv_ptr, int p1x, int p1y, int p2x, int p2y){
  int p1r = (int)cv_ptr->image.at<Vec3b>(Point(p1x,p1y))[0];
  int p1g = (int)cv_ptr->image.at<Vec3b>(Point(p1x,p1y))[1];
  int p1b = (int)cv_ptr->image.at<Vec3b>(Point(p1x,p1y))[2];

  int p2r = (int)cv_ptr->image.at<Vec3b>(Point(p2x,p2y))[0];
  int p2g = (int)cv_ptr->image.at<Vec3b>(Point(p2x,p2y))[1];
  int p2b = (int)cv_ptr->image.at<Vec3b>(Point(p2x,p2y))[2];

  p1r = pow(p1r - p2r,2.0);
  p1g = pow(p1g - p2g,2.0);
  p1b = pow(p1b - p2b,2.0);
  return sqrt(p1r+p1g+p1b);

}

Point findCenter(cv_bridge::CvImagePtr cv_ptr, Point A, Point B, Point C){
  Point center;
    float yDelta_a = B.y - A.y;
    float xDelta_a = B.x - A.x;
    float yDelta_b = C.y - B.y;
    float xDelta_b = C.x - B.x;

    float aSlope = yDelta_a/xDelta_a;
    float bSlope = yDelta_b/xDelta_b;  

    center.x = cv_ptr->image.cols;
    center.y = cv_ptr->image.rows;

    center.x = (aSlope*bSlope*(A.y - C.y) + bSlope*(A.x + B.x)- aSlope*(B.x+C.x) )/(2* (bSlope-aSlope) );
    if(aSlope < .01){
      center.y = -1 * (center.x - (B.x + C.x) / 2) / bSlope +  (B.y + C.y) / 2;
    }
    else{
      center.y = -1*(center.x - (A.x+B.x)/2)/aSlope +  (A.y+B.y)/2;
    }

  return center;
}

int findDistance(int p1x, int p1y, int p2x, int p2y){
  double dx = abs(p1x - p2x);
  double dy = abs(p1y - p2y);
  return sqrt(pow(dx,2.0)+pow(dy,2.0));
}

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

    //###############Add code between here############################
  int centerX = cv_ptr->image.cols/2;
  int centerY = cv_ptr->image.rows/2;
  int currX = centerX;
  int currY = centerY;
  edgeCount = 0;
  foundEdge=false;
  
  while(currX < cv_ptr->image.cols && !foundEdge){
    if(colorDifference(cv_ptr,currX,currY,currX+5,currY) > 200){
      pointList.push_back(Point(currX+5,currY));
      foundEdge=true;
      edgeCount++;
    }
    currX++;
  }

  foundEdge=false;
  currX=centerX;
  while(currX > 5 && !foundEdge){
    if(colorDifference(cv_ptr,currX,currY,currX-5,currY) > 200){
      pointList.push_back(Point(currX-5,currY));
      foundEdge=true;
      edgeCount++;
    }
    currX--;
  }

  foundEdge=false;
  currX = centerX;
  while(currY < cv_ptr->image.rows - 5 && !foundEdge){
    if(colorDifference(cv_ptr,currX,currY,currX,currY+5) > 200){
      pointList.push_back(Point(currX,currY+5));
      foundEdge=true;
      edgeCount++;
    }
    currY++;
  }
  
  while(pointList.size() > 0){
    Point individual = pointList.back();
    int x = individual.x;
    int y = individual.y;
    circle(cv_ptr->image,Point(x,y),0,colorScalar_RED,1);
    pointList2.push_back(individual);
    pointList.pop_back();
  }
  if(edgeCount == 3){
    Point p1 = pointList2.back();
    pointList2.pop_back();
    Point p2 = pointList2.back();
    pointList2.pop_back();
    Point p3 = pointList2.back();
    pointList2.pop_back();
    Point centre = findCenter(cv_ptr,p1,p2,p3);
    double radius = findDistance(p1.x,p1.y,centre.x,centre.y);
    radius = abs(radius);
    cout<<radius<<endl;
    cout<<centre<<endl;
    circle(cv_ptr->image,centre,radius,colorScalar_RED,1);
    cout<<"Found circle, Drawing it!"<<endl;
    circle(cv_ptr->image,Point(centerX,centerY),4,colorScalar_RED,3);
  }
  else{
    cout<<edgeCount<<endl;
    cout<<"No circle found!"<<endl;
  }
    //########## AND HERE #######################
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
