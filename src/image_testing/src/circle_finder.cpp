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

const int testPointX=200;
const int testPointY=100;

const int accuracy = 200;

int edgeCount= 0;
//BGR
Scalar colorScalar_RED = Scalar(0,0,255);
Scalar colorScalar = Scalar(0,255,0);

int window_scalar = 5;
vector<Point> pointList;
vector<Point> pointList2;

Mat colorImage;

vector<Vec3f> circles;

Point region_corner = Point(0,0);
int region_width =0;
int region_height = 0;

bool foundEdge=false;




class ImageConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub_color;
    ros::Publisher circInf;
    point_message::pointMsg circleInfo;
    ros::Subscriber regionInfo;

public:
    ImageConverter()
            : it(nh)
    {
        image_sub = it.subscribe("/camera/image_raw", 1,
                                 &ImageConverter::imageCb, this);

        image_pub = it.advertise("/camera/search_region", 1);
        circInf = nh.advertise<point_message::pointMsg>("circleInfo",1);
        regionInfo = nh.subscribe<point_message::statsMsg>("objectColorInfo",1, &ImageConverter::regionInfoCb, this);
    }

    ~ImageConverter()
    {}
    void regionInfoCb(point_message::statsMsg regionInfo){
        region_corner = Point(regionInfo.xCent - window_scalar * regionInfo.xSD,
                              regionInfo.yCent - window_scalar * regionInfo.ySD);
        region_width = 2 * window_scalar * regionInfo.xSD;
        region_height = 2 * window_scalar * regionInfo.ySD;
    }
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
        im_gray = cv_ptr->image(Rect(region_corner.x,region_corner.y,
                                region_width,
                                region_height));

        Mat search_region;
        im_gray.copyTo(search_region);
        HoughCircles( search_region, circles, CV_HOUGH_GRADIENT, 1.56, 90, 160, 70, 50, 0 );
        //ROS_INFO("%3f",circles.size());
        /*
      for( size_t i = 0; i < circles.size(); i++ )
      {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
        //ROS_INFO("Made circle!");
      }
         */
        /*
        circleInfo.xCoor = circles[0][0] + region_corner.x;
        circleInfo.yCoor = circles[0][1] + region_corner.y;
        circleInfo.radius = circles[0][2];
         */
        circleInfo.xCoor = circles[0][0];
        circleInfo.yCoor = circles[0][1];
        circleInfo.radius = circles[0][2];

        circInf.publish(circleInfo);

        cv_ptr->image = search_region;
        image_pub.publish(cv_ptr->toImageMsg());

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "draw_gui");
    ImageConverter ic;
    ros::spin();
    return 0;
}