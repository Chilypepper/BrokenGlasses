#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctime>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include "imu_3dm_gx4/FilterOutput.h"
#define PI 3.14159

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
    ros::Subscriber pitch_sub;
    ros::Subscriber roll_sub;


public:
    ImageConverter()
            : it(nh)
    {
        image_sub = it.subscribe("/camera/image_raw", 1,
                                 &ImageConverter::imageCb, this);
        image_pub = it.advertise("/camera/horizon_line", 1);
        pitch_sub = nh.subscribe("pitchInput",1,&ImageConverter::pitchCB,this);
        roll_sub = nh.subscribe("rollInput",1,&ImageConverter::rollCB,this);
        imu_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>("/imu/filter", 1, &ImageConverter::imuCB, this);
    }

    ~ImageConverter()
    {}
    void imuCB(const imu_3dm_gx4::FilterOutput::ConstPtr& filter){
        float x,y,z,w;
        x=filter->orientation.x;
        y=filter->orientation.y;
        z=filter->orientation.z;
        w=filter->orientation.w;
        pitch = asin(2*(y*w-x*z));
        roll = atan((2*(x*w+y*z))/(x*x+y*y-z*z-w*w));
    }
    void pitchCB(const std_msgs::Float32& pitched){
        pitch = pitched.data;
    }
    void rollCB(const std_msgs::Float32& rolled){
        roll = rolled.data;
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
        float midY = -1 * (pitch/(PI/6))*(play.rows/2);
        midY += play.rows/2;
        Point centerOfLine = Point(play.cols/2,midY);
        //line(play,Point(0,midY),Point(play.cols,midY),Scalar(0,0,255),3);
        putText(play,"Horizon Line",Point(150,midY-15),3,.8,Scalar(0,0,255));

        //roll assumes 0 is normal, clockwise roll is > 0
        Point rightPt;
        Point leftPt;
        
        rightPt.y = midY;
        rightPt.y -= 100 * sin(roll);
        rightPt.x = play.cols / 2;
        rightPt.x += 100 * cos(roll);

        leftPt.y = midY;
        leftPt.y += 100 * sin(roll);
        leftPt.x = play.cols / 2;
        leftPt.x -= 100 * cos(roll);

        line(play,leftPt,rightPt,Scalar(0,255,0),3);
        //line(play,Point(play.cols/2,midY),leftPt,Scalar(255,0,255),3);
        ROS_INFO("Pitch: %5f",pitch);
        ROS_INFO("Roll: %5f",roll);
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
