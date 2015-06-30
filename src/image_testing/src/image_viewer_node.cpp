#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


using namespace cv_bridge;
using namespace cv;
class Viewer
{
  private:
    ros::NodeHandle nh;
  public:
    Viewer() :nh(){
    	Mat image;
    	cvLoadImage("~/frame0000.jpg",CV_LOAD_IMAGE_COLOR);
    	cvNamedWindow("Disp");
    	cvWaitKey(0);
    }    
    void loop()
    {
      ros::Rate rate(1);
      while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
      }
    }
};
int main(int argc, char **argv)
{
  ros::init(argc,argv,"image_viewer");
  Viewer viewer;
  viewer.loop();
}
