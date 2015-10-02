//
// Created by peter on 9/30/15.
//

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


#undef drawCircle

using namespace cv;
using namespace std;

const int MARKER_NUM_X = 13;
const int MARKER_NUM_Y = 13;

const int MARKER_HEIGHT = 20;
//grid thickness
const int THICKNESS = 2;

const int vertexYAdjust = 60;
const int verteX = 70;


int edgeCount= 0;
//RGB
Scalar RED = Scalar(255,0,0);
Scalar NEON_GREEN = Scalar(0,255,0);
Scalar NEON_BLUE = Scalar(0,0,255);

Scalar colorScalar = Scalar(0,255,0);

vector<Vec3f> circles;

int detectedCircleX = 0;
int detectedCircleY = 0;
int detectedCircleR = 0;

int objectColorX = 0;
int objectColorY = 0;


bool foundEdge=false;

void clear(Mat &raw, int &detectedCircleR, int  &detectedCircleX, int &detectedCircleY){
    detectedCircleR = 0;
    detectedCircleX = raw.cols/2;
    detectedCircleY = raw.rows/2;
}
void drawGrid(Mat &raw){
    int rawRows = raw.rows;
    int rawCols = raw.cols;
    Point leftCent = Point(0, rawRows/2);
    Point rightCent = Point(rawCols, rawRows/2);
    Point topCent = Point(rawCols/2, 0);
    Point bottomCent = Point(rawCols/2, rawRows);
    //draw x axis
    line(raw, leftCent, rightCent, RED, THICKNESS);
    //draw y axis
    line(raw, topCent, bottomCent, RED, THICKNESS);
    //draw x-axis markers
    int xInc = rawCols / MARKER_NUM_X;
    for(int i = 1; i < MARKER_NUM_X ; i++){
        Point low = Point(i * xInc, rawRows/2 - MARKER_HEIGHT);
        Point high = Point(i * xInc, rawRows/2 + MARKER_HEIGHT);
        line(raw, low, high, RED, THICKNESS);
    }
    //draw y axis markers
    int yInc = rawRows / MARKER_NUM_Y;
    for(int i = 1; i < MARKER_NUM_X ; i++){
        Point low = Point(rawCols/2 - MARKER_HEIGHT, i * yInc);
        Point high = Point(rawCols/2 + MARKER_HEIGHT, i * yInc);
        line(raw, low, high, RED, THICKNESS);
    }
}
void drawXYBars(Mat &raw){
    Point recCorn1 = Point(25,25);
    Point vertex = Point(verteX, raw.rows-vertexYAdjust);

    rectangle(raw,recCorn1,vertex, RED);

    Point recCorn3 = Point(raw.cols - 25,raw.rows - 25);
    rectangle(raw,recCorn3, vertex, RED);
    //draw y coordinate
    Point midY = Point(25,raw.rows/2);
    rectangle(raw, midY, Point(vertex.x,detectedCircleY), RED,-1);
    Point midX = Point(raw.cols/2, raw.rows - 25);
    rectangle(raw, midX, Point(detectedCircleX, raw.rows-vertexYAdjust), RED,-1);
}

class ImageConverter
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    ros::Subscriber circleSub;
    point_message::pointMsg circlePoint;
    point_message::pointMsg objectColorInfo;
    ros::Subscriber objectColorSub;

public:
    ImageConverter()
            : it(nh)
    {
        image_sub = it.subscribe("/camera/image_raw", 1,
                                 &ImageConverter::imageCb, this);
        image_pub = it.advertise("/camera/output_video_gui", 1);
        circleSub = nh.subscribe<point_message::pointMsg>("circleInfo",1, &ImageConverter::updateCircleInfo, this);
        objectColorSub = nh.subscribe<point_message::statsMsg>("objectColorInfo",
                                                               1,
                                                               &ImageConverter::updateColorInfo,
                                                               this);
    }

    ~ImageConverter()
    {}
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        Mat &raw = cv_ptr->image;
        Mat drawing(raw.rows,raw.cols,CV_8UC3, Scalar(0,0,0));


        drawGrid(drawing);

        circle(drawing,Point(detectedCircleX,detectedCircleY),detectedCircleR,RED);
        circle(drawing,Point(detectedCircleX,detectedCircleY),5, NEON_GREEN, -1);
        circle(drawing,Point(objectColorX,objectColorY),5,NEON_BLUE,-1);

        drawXYBars(drawing);
        double a =0.2;
        double b = 1.0 - a;
        addWeighted(raw,a,drawing,b,0.0,raw);

        //clear(raw, detectedCircleR, detectedCircleX, detectedCircleY);
        image_pub.publish(cv_ptr->toImageMsg());
    }
    void updateCircleInfo(const point_message::pointMsg circArr){
        detectedCircleX = circArr.xCoor;
        detectedCircleY = circArr.yCoor;
        detectedCircleR = circArr.radius;
    }
    void updateColorInfo(const point_message::statsMsg objectColorInfo){

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}