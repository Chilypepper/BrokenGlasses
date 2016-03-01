#include <opencv2/imgproc/imgproc.hpp>
#include </home/peter/brokenGlasses/src/image_testing/libs/RiptideVision.h>

using namespace cv;
using namespace std;

struct colorPoint{
  int x;
  int y;
};

Mat RiptideVision::seperateColors(Mat src, vector<int> colors)
{
  Mat input = src.clone();
  Mat imageHSV;
  Mat imgThreshold;

  cvtColor( input, imageHSV, COLOR_BGR2HSV );

  switch(colors[6]){
    case 0:
        inRange(imageHSV, Scalar(colors[0], colors[2], colors[4]), Scalar(colors[1], colors[3], colors[5]), imgThreshold);
    break;

  }
  //Add red secondary threshhold
  return imgThreshold;
}

void RiptideVision::colorAverage(Mat src, vector<int> colors, colorPoint averagePoint){
  Mat M = seperateColors(src,colors);
  averagePoint.x = 0;
  averagePoint.y = 0;

  long long iSum = 0;
  long long jSum = 0;
  int count = 0;

  for(int i = 0; i < M.rows; i++){
    const double* Mi = M.ptr<double>(i);
    for(int j = 0; j < M.cols; j++)
      if(Mi[j] > 0){
      iSum += i;
      jSum += j;
      count++;
    }
  }
}


int main(){

}