#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace cv_bridge;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    ROS_INFO("Step 1");
    
    ROS_INFO("SETUP");

    namedWindow(OPENCV_WINDOW);
  }
  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    CvImagePtr cv_ptr;
    ROS_INFO("\033[1;31mbold red text\033[0m\n");
    ROS_INFO("Step2");
    
    try
    {
      ROS_INFO("Trying to convert");
      cv_ptr = toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      ROS_INFO("Converted");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_INFO("Not Converted");

      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ROS_INFO("Step 3");
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));
    // Update GUI Window
    //imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
