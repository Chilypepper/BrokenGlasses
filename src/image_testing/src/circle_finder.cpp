#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctime>
#include <point_message/pointMsg.h>

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

Mat colorImage;

vector<Vec3f> circles;


bool foundEdge=false;

class ImageConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub_color;
    ros::Publisher pointPub;
    point_message::pointMsg circlePoint;

public:
    ImageConverter()
            : it(nh)
    {
      image_sub = it.subscribe("/camera/image_raw", 1,
                               &ImageConverter::imageCb, this);
      image_sub_color = it.subscribe("/camera/image_raw", 1,
                                     &ImageConverter::imageCb, this);
      image_pub = it.advertise("/camera/output_video", 1);
      pointPub = nh.advertise<point_message::pointMsg>("circlePoint",1);
    }

    ~ImageConverter()
    {}
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      circles.clear();
      Mat im_gray;
      //cvtColor( cv_ptr->image, im_gray, CV_BGR2GRAY);
      GaussianBlur(cv_ptr->image, im_gray, Size(9, 9), 2, 2 );

      HoughCircles( im_gray, circles, CV_HOUGH_GRADIENT, 1.80, 100, 100, 90, 0, 0 );
      ROS_INFO("%3f",circles.size());

      for( size_t i = 0; i < circles.size(); i++ )
      {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
        ROS_INFO("Made circle!");
      }

      circlePoint.xCoor = circles[0][0];
      circlePoint.yCoor = circles[0][1];
      pointPub.publish(circlePoint);
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

/*
 * #include <ros/ros.h>
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
  }

  ~ImageConverter()
  {}
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    Mat im_gray;
    //cvtColor( cv_ptr->image, im_gray, CV_BGR2GRAY );
    GaussianBlur(im_gray, im_gray, Size(9, 9), 2, 2 );
    vector<Vec3f> circles;
    //###############Add code between here############################
    HoughCircles( im_gray, circles, CV_HOUGH_GRADIENT, 1, im_gray.rows/20, 200, 100, 0, 0 );
    //########## AND HERE #######################
    for( size_t i = 0; i < circles.size(); i++ )
    {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
      ROS_INFO("Made circle!");
    }
    cv_ptr->image = im_gray;
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

 */
