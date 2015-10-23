#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctime>
#include "geometry_msgs/Vector3.h"

using namespace cv;
using namespace std;

float pitch = 0 ;
float roll = 0;
float compass = 0;

class ImageConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    ros::Subscriber imu_sub;

public:
    ImageConverter()
            : it(nh)
    {
        image_sub = it.subscribe("/camera/image_raw", 1,
                                 &ImageConverter::imageCb, this);
        image_pub = it.advertise("/camera/horizon_line", 1);
        imu_sub = nh.subscribe("eulerAngles",1,&ImageConverter::imuCB,this);
    }

    ~ImageConverter()
    {}
    void imuCB(const geometry_msgs::Vector3& angleSet){
        //update values
        pitch = angleSet.y;
        roll = angleSet.z;
        compass = angleSet.x;
    }
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
        //code assumes FOV of 30 degrees above or below center and that msg is in degrees.
        Mat play = cv_ptr->image;
        float midY = (pitch / 30.0) + play.rows;
        Point centerOfLine = Point(play.cols/2,midY);
        

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
