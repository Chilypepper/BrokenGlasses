#include <opencv2/imgproc/imgproc.hpp>
#include </home/peter/brokenGlasses/src/image_testing/libs/RiptideVision.h>

using namespace cv;
using namespace std;


void RiptideVision::seperateColors(Mat src, vector<int> colors, Mat &imgThreshold){
  Mat input = src.clone();
  GaussianBlur(input,input,Size(21,21),0);
  Mat imageHSV;

  cvtColor( input, imageHSV, COLOR_BGR2HSV );

  switch(colors[6]){
    case 0:
        inRange(imageHSV, Scalar(colors[0], colors[2], colors[4]), Scalar(colors[1], colors[3], colors[5]), imgThreshold);
    break;
  }
  cvtColor(imgThreshold,imgThreshold,CV_GRAY2BGR);
}

void RiptideVision::colorAverage(Mat src, vector<int> colors, Point &averagePoint){
  Mat M;
  seperateColors(src,colors, M);
  averagePoint.x = 0;
  averagePoint.y = 0;

  long long int iSum = 0;
  long long int jSum = 0;
  int count = 1;

  for(int i = 0; i < M.cols - 1; i++){
    for(int j = 0; j < M.rows - 1; j++){
      if(M.at<uchar>(j,i) > 0){
        iSum += i;
        jSum += j;
        count++;
      }
    }
  }
  averagePoint.x = iSum / count;
  averagePoint.y = jSum / count;
}
void RiptideVision::orientation(Mat src, vector<int> colors, Point averagePoint, linePoint &pair){
  Mat M;
  seperateColors(src,colors, M);

  long long int iSumTop = 0;
  long long int jSumTop = 0;

  long long int iSumBot = 0;
  long long int jSumBot = 0;

  long long int count = 1;

  //**Top half
  for(int i = 0; i < M.cols - 1; i += 2){
    for(int j = 0; j < averagePoint.y - 1; j += 2){
      if(M.at<uchar>(j,i) > 0){
        iSumTop += i;
        jSumTop += j;
        count++;
      }
    }
  }
  pair.top.x = iSumTop / count;
  pair.top.y = jSumTop / count;

  count = 1;

  //** Bottom half
  for(int i = 0; i < M.cols - 1; i++){
    for(int j = averagePoint.y; j < M.rows - 1; j++){
      if(M.at<uchar>(j,i) > 0){
        iSumBot += i;
        jSumBot += j;
        count++;
      }
    }
  }
  pair.bot.x = iSumBot / count;
  pair.bot.y = jSumBot / count;
}

void RiptideVision::buoyTask(Mat src, buoyInfo feedback,Mat &drawing){
  Point red;
  colorAverage(src,REDS,red);
  Point green;
  colorAverage(src,GREENS,green);
  Point yellow;
  colorAverage(src,YELLOWS,yellow);
  circle(drawing,red,3,PINK,-1);
  circle(drawing,green,3,PINK,-1);
  circle(drawing,yellow,3,PINK,-1);
}

int main(){

}