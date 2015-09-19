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
const int blueLowerBound = 20;
const int blueUpperBound = 50;
const int greenLowerBound = 100;
const int greenUpperBound = 130;
const int redLowerBound = 100;
const int redUpperBound = 130;

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
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        Scalar lower_yellow = Scalar(0,80,80);
        Scalar upper_yellow = Scalar(100,200,200);
        Mat hsv(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);

        cvtColor(cv_ptr->image, hsv, COLOR_BGR2HSV);

        /*Mat mask;
        inRange(hsv,lower_yellow,upper_yellow,mask);

        bitwise_and(cv_ptr->image, cv_ptr->image, cv_ptr->image, mask);
        */
         cv_ptr->image = hsv;
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
