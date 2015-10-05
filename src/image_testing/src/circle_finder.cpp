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

int window_scalar = 3;
vector<Point> pointList;
vector<Point> pointList2;

Mat colorImage;

vector<Vec3f> circles;

Point region_corner = Point(0,0);
int region_width =0;
int region_height = 0;

bool foundEdge=false;
bool hasNewVals = true;

Mat search_region2;
Mat im_gray;


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
        image_sub = it.subscribe("/camera/output_video", 1,
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
        hasNewVals = true;
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
        //cvtColor( cv_ptr->image, im_gray, CV_BGR2GRAY);
        GaussianBlur(cv_ptr->image, im_gray, Size(9, 9), 2, 2 );
        /*im_gray = cv_ptr->image(Rect(region_corner.x,region_corner.y,
                                region_width,
                                region_height));

    */
        bool hasRegion = true;
        int xUpperRange = 1;
        int yUpperRange = 1;
        if(hasNewVals) {
            int xUpperRange = region_width;
            int yUpperRange = region_height;
            /*
            ROS_INFO("%3i height", im_gray.rows);
            ROS_INFO("%3i wiedth", im_gray.cols);
            ROS_INFO("%3i x corner", region_corner.x);
            ROS_INFO("%3i y corner", region_corner.y);
            ROS_INFO("%3i x Upper", xUpperRange);
            ROS_INFO("%3i y Upper", yUpperRange);
            ROS_INFO("%3i y combo", yUpperRange + region_corner.y);
            ROS_INFO("%3i x combo", xUpperRange + region_corner.x);
            ROS_INFO("%s","AFTER");
             */

            if (region_corner.x < 0) {
                region_corner.x = 1;
            }
            if (region_corner.y < 0) {
                region_corner.y = 1;
            }
            if (region_corner.x > im_gray.cols) {
                region_corner.x = im_gray.cols - 1;
            }
            if (region_corner.y > im_gray.rows) {
                region_corner.y = im_gray.rows - 1;
            }
            if (region_corner.x + xUpperRange > im_gray.cols) {
                xUpperRange = im_gray.cols - region_corner.x - 1;
            }
            /*else {
                xUpperRange = region_corner.x + xUpperRange;
            }*/
            if (region_corner.y + yUpperRange > im_gray.rows) {
                yUpperRange = im_gray.rows - region_corner.y - 1;
            }
            /*else {
                yUpperRange = region_corner.y + yUpperRange;
            }

            ROS_INFO("%3i height", im_gray.rows);
            ROS_INFO("%3i wiedth", im_gray.cols);
            ROS_INFO("%3i x corner", region_corner.x);
            ROS_INFO("%3i y corner", region_corner.y);
            ROS_INFO("%3i x Upper", xUpperRange);
            ROS_INFO("%3i y Upper", yUpperRange);
            ROS_INFO("%3i y combo", yUpperRange + region_corner.y);
            ROS_INFO("%3i x combo", xUpperRange + region_corner.x);
             */

            Mat search_region(im_gray, Rect(region_corner.x,
                                            region_corner.y,
                                            xUpperRange,
                                            yUpperRange));
            search_region.copyTo(search_region2);
            hasRegion = true;
        }
        if(search_region2.rows > 0) {
            //ROS_INFO("%s","Fails @ circle");
            //ROS_INFO("%3i",search_region2.rows);
            HoughCircles(search_region2, circles, CV_HOUGH_GRADIENT, 2.10, 1, 200, 80, 30, 0);
            //ROS_INFO("%s","Fails past circle");
            ROS_INFO("%3i",circles.size());


        }
        else {
            HoughCircles(im_gray, circles, CV_HOUGH_GRADIENT, 1.60, 90, 120, 70, 0, 0);
        }
        //ROS_INFO("%3f",circles.size());



        if(circles.size() > 0) {
            circleInfo.xCoor = region_corner.x + circles[0][0];
            circleInfo.yCoor = region_corner.y + circles[0][1];
            circleInfo.radius = circles[0][2];
        }



        circInf.publish(circleInfo);

        cv_ptr->image = search_region2;
        hasNewVals = true;
        image_pub.publish(cv_ptr->toImageMsg());

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_finder");
    ImageConverter ic;
    ros::spin();
    return 0;
}