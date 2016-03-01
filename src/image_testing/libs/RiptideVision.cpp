#include <opencv2/imgproc/imgproc.hpp>
#include </home/peter/brokenGlasses/src/image_testing/libs/RiptideVision.h>

using namespace cv;
using namespace std;

struct colorPoint{
  int x;
  int y;
};
struct linePoint{
  colorPoint top;
  colorPoint bot;
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

void RiptideVision::colorAverage(Mat src, vector<int> colors, colorPoint &averagePoint){
  Mat M = seperateColors(src,colors);
  averagePoint.x = 0;
  averagePoint.y = 0;

  long long int iSum = 0;
  long long int jSum = 0;
  int count = 0;

  for(int i = 0; i < M.cols;i += 2){
    for(int j = 0; j < M.rows;j += 2){
      //cout << i << " " << j << endl;
      if(M.at<uchar>(i,j) > 0){
        iSum += i;
        jSum += j;
        count++;
      }
    }
  }
  averagePoint.x = jSum / count;
  averagePoint.y = iSum / count;
}
void RiptideVision::orientation(Mat src, vector<int> colors, colorPoint averagePoint, linePoint &pair){
  Mat M = seperateColors(src,colors); 

  averagePoint.x = 0;
  averagePoint.y = 0;

  long long int iSumTop = 0;
  long long int jSumTop = 0;

  long long int iSumBot = 0;
  long long int jSumBot = 0;

  int count = 0;

  //**Top half
  for(int i = 0; i < M.cols; i += 2){
    for(int j = 0; j < averagePoint.y; j += 2){
      //cout << i << " " << j << endl;
      if(M.at<uchar>(i,j) > 0){
        iSumTop += i;
        jSumTop += j;
        count++;
      }
    }
  }
  pair.top.x = iSumTop / count;
  pair.top.y = jSumTop / count;

  count = 0;

  //** Bottom half
  for(int i = 0; i < M.cols; i += 2){
    for(int j = averagePoint.y; j < M.rows; j += 2){
      //cout << i << " " << j << endl;
      if(M.at<uchar>(i,j) > 0){
        iSumBot += i;
        jSumBot += j;
        count++;
      }
    }
  }
  pair.bot.x = iSumBot / count;
  pair.bot.y = jSumBot / count;

}

int main(){

}