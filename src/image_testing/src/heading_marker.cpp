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
        image_pub = it.advertise("/camera/output_video_flipped", 1);
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

        Mat marker = cv_ptr->image;
        int redUpper = 165;
        int redLower = 10;
        int satUpper = 210;
        int satLower = 150;
        int valUpper = 240;
        int valLower = 150;

        bool hasHue = false;
        bool hasSat = false;
        bool hasVal = false;
        Vec3b white = {255,255,255};

        Mat blacknwhite = Mat::zeros(marker.rows,marker.cols,CV_64F);

        for(int row = 0; row < blacknwhite.rows; row++){
            for(int col = 0; col < blacknwhite.cols; col++){
                Vec3b hsvValues = marker.at<Vec3b>(Point(row,col));
                int hue = hsvValues[0];
                int saturation = hsvValues[1];
                int brightness = hsvValues[2];
                //red is special case given its hue values
                if(hue > redUpper || hue < redLower){
                    /*
                     * checks HSV conditions and determines if all are true
                     */
                    hasHue = true;
                    if(saturation < satUpper && saturation < satLower){
                        hasSat = true;
                        if(brightness < valUpper && brightness > valLower){
                            hasVal = true;
                        }
                    }
                }
                //assumes all true
                if(hasHue && hasSat && hasVal){
                    blacknwhite
                }

            }
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
