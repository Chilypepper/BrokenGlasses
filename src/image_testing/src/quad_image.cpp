#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctime>
#include <point_message/pointMsg.h>
#include <point_message/statsMsg.h>

using namespace cv;
using namespace std;

Mat stitched = Mat::zeros(964*2,1288*2,CV_8UC3);
int ch = stitched.channels();

class ImageConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub1;
    image_transport::Subscriber image_sub2;
    image_transport::Subscriber image_sub3;
    image_transport::Subscriber image_sub4;

public:
    ImageConverter()
            : it(nh)
    {
        image_sub1 = it.subscribe("/camera/image_raw", 1,
                                 &ImageConverter::imageCb1, this);
        image_sub2 = it.subscribe("/camera/output_video_gui", 1,
                                  &ImageConverter::imageCb2, this);
        image_sub3 = it.subscribe("/camera/output_video", 1,
                                  &ImageConverter::imageCb3, this);
        image_sub4 = it.subscribe("/camera/search_region", 1,
                                  &ImageConverter::imageCb4, this);

        image_pub = it.advertise("/camera/stitched", 1);
    }
    cv_bridge::CvImagePtr stitch_ptr;

    ~ImageConverter()
    {}
    void imageCb1(const sensor_msgs::ImageConstPtr& msg)
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

        cv_ptr->image.copyTo(stitched.rowRange(0,cv_ptr->image.rows).colRange(0,cv_ptr->image.cols));
        cv_ptr->image = stitched;
        image_pub.publish(cv_ptr->toImageMsg());
    }
    void imageCb2(const sensor_msgs::ImageConstPtr& msg)
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

        cv_ptr->image.copyTo(stitched.rowRange(0,cv_ptr->image.rows).colRange(1288, 1288+cv_ptr->image.cols));
    }
    void imageCb3(const sensor_msgs::ImageConstPtr& msg)
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
        cv_ptr->image.copyTo(stitched.rowRange(964,964+cv_ptr->image.rows).colRange(0, cv_ptr->image.cols));


    }
    void imageCb4(const sensor_msgs::ImageConstPtr& msg)
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
        cv_ptr->image.copyTo(stitched.rowRange(964,964+cv_ptr->image.rows).colRange(1288, 1288+cv_ptr->image.cols));


    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quad_image");
    ImageConverter ic;
    ros::spin();
    return 0;
}