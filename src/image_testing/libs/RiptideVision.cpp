#include <opencv2/imgproc/imgproc.hpp>
#include </home/peter/riptideVision/RiptideVision.h>

using namespace cv;
using namespace std;

Mat RiptideVision::seperateColors(Mat src, vector<int> colors)
{
    Mat input = src.clone();
    Mat imageHSV;
    Mat imgThreshold;

    cvtColor( input, imageHSV, COLOR_BGR2HSV );

    inRange(imageHSV, Scalar(colors[0], colors[2], colors[4]), Scalar(colors[1], colors[3], colors[5]), imgThreshold);
    //Add red secondary threshhold
    return imgThreshold;
}


int main(){

}