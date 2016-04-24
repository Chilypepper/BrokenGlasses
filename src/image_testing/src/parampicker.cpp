#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctime>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include </home/peter/brokenglasses/src/image_testing/libs/RiptideVision.h>
//#include <RiptideVision.h>
#include <vector>

using namespace cv;
using namespace std;

int iteration = 0;

int correlation_window_size = 5;
int disparity_range = 32;
int min_disparity = -128;
int prefilter_cap = 1;
int prefilter_size = 5;
int speckle_range = 0;
int speckle_size = 0;
int texture_threshold = 0;
int uniqueness_ratio = 0;
bool found = false;

/*
[ INFO] [1458427884.261557081]: 423955
[ INFO] [1458427884.354750908]: 423956
[ INFO] [1458427884.376414520]: 36
[ INFO] [1458427884.376470242]: 163
[ INFO] [1458427884.376490563]: 19
[ INFO] [1458427884.376505442]: 53
[ INFO] [1458427884.376521451]: 146
[ INFO] [1458427884.376535629]: 20
[ INFO] [1458427884.376553198]: 24
[ INFO] [1458427884.376566338]: 76
[ INFO] [1458427884.376580115]: 4
[ INFO] [1458427884.376595101]: 423956
[ INFO] [1458428962.445351542]: 36
[ INFO] [1458428962.445491078]: 163
[ INFO] [1458428962.445607897]: 19
[ INFO] [1458428962.445667731]: 53
[ INFO] [1458428962.445713924]: 146
[ INFO] [1458428962.445792935]: 20
[ INFO] [1458428962.445831596]: 24
[ INFO] [1458428962.445866461]: 76
[ INFO] [1458428962.445937817]: 4
[ INFO] [1458428962.445973396]: 449831
*/
class ImageConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    image_transport::Subscriber disp_sub;
    ros::Subscriber sub;

public:
    ImageConverter()
    : it(nh)
    {
        image_sub = it.subscribe("/stereo/left/image_rect_color", 1,
            &ImageConverter::imageCb, this);
        //disp_sub = it.subscribe("/stereo/disparity", 1,
        //    &ImageConverter::gotDisp,this);
    }
    ~ImageConverter()
    {}
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        iteration++;
        if(!found){
            srand(time(NULL));
            correlation_window_size = 5 + rand() % 249;
            disparity_range = 32 + rand() % 200;
            min_disparity = -128 + rand() % 250;
            prefilter_cap = 1 + rand() % 60;
            prefilter_size = 5 + rand() % 200;
            speckle_size = rand() % 97;
            speckle_range = rand() % 31;
            texture_threshold = rand() % 20;
            uniqueness_ratio = rand() % 5;

            nh.setParam("/stereo/stereo_image_proc/correlation_window_size", correlation_window_size);
            nh.setParam("/stereo/stereo_image_proc/disparity_range", disparity_range);
            nh.setParam("/stereo/stereo_image_proc/min_disparity", min_disparity);
            nh.setParam("/stereo/stereo_image_proc/prefilter_cap", prefilter_cap);
            nh.setParam("/stereo/stereo_image_proc/prefilter_size", prefilter_size);
            nh.setParam("/stereo/stereo_image_proc/speckle_range", speckle_range);
            nh.setParam("/stereo/stereo_image_proc/speckle_size", speckle_size);
            nh.setParam("/stereo/stereo_image_proc/texture_threshold", texture_threshold);
            nh.setParam("/stereo/stereo_image_proc/uniqueness_ratio", uniqueness_ratio);
            ROS_INFO("%i",iteration);
        }
    }

};
void gotDisp(const stereo_msgs::DisparityImage& msg)
    {
        ros::NodeHandle nh;

        nh.getParam("/stereo/stereo_image_proc/correlation_window_size", correlation_window_size);
        nh.getParam("/stereo/stereo_image_proc/disparity_range", disparity_range);
        nh.getParam("/stereo/stereo_image_proc/min_disparity", min_disparity);
        nh.getParam("/stereo/stereo_image_proc/prefilter_cap", prefilter_cap);
        nh.getParam("/stereo/stereo_image_proc/prefilter_size", prefilter_size);
        nh.getParam("/stereo/stereo_image_proc/speckle_range", speckle_range);
        nh.getParam("/stereo/stereo_image_proc/speckle_size", speckle_size);
        nh.getParam("/stereo/stereo_image_proc/texture_threshold", texture_threshold);
        nh.getParam("/stereo/stereo_image_proc/uniqueness_ratio", uniqueness_ratio);

        ROS_INFO("%i", correlation_window_size);
        ROS_INFO("%i", disparity_range);
        ROS_INFO("%i", min_disparity);
        ROS_INFO("%i", prefilter_cap);
        ROS_INFO("%i", prefilter_size);
        ROS_INFO("%i", speckle_range);
        ROS_INFO("%i", speckle_size);
        ROS_INFO("%i", texture_threshold);
        ROS_INFO("%i", uniqueness_ratio);
        ROS_INFO("%i",iteration);
        found = true;
    }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "paramset");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/stereo/disparity", 1, gotDisp);

    ImageConverter ic;
    ros::spin();
    return 0;
}
