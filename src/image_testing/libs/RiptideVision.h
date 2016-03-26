
const int redsArr[] = {155,180,80,200,100,255,0};
const std::vector<int> REDS (redsArr, redsArr + sizeof(redsArr) / sizeof(int));

const int greensArr[] = {60, 75,30,100,60,180,0};
const std::vector<int> GREENS (greensArr, greensArr + sizeof(greensArr) / sizeof(int));

const int yellowsArr[] = {25,35,50,255,80,255,0};
const std::vector<int> YELLOWS (yellowsArr, yellowsArr + sizeof(yellowsArr) / sizeof(int));

const int blaze_orange_arr[] = {5,34,30,150,150,255};
const std::vector<int> BLAZE_ORANGE (blaze_orange_arr, blaze_orange_arr + sizeof(blaze_orange_arr) / sizeof(int));

const cv::Scalar PINK = cv::Scalar(255,0,255);
class RiptideVision{

	public:

		struct buoyInfo{
			bool yellowB;
			bool redB;
			bool greenB;

			cv::Mat drawing;
		};
		struct linePoint{
		  cv::Point top;
		  cv::Point bot;
		};
		static void seperateColors(cv::Mat, std::vector<int>, cv::Mat&);
		static void colorAverage(cv::Mat, std::vector<int>, cv::Point&);
		static void orientation(cv::Mat, std::vector<int>, cv::Point, linePoint&);
		static void buoyTask(cv::Mat, buoyInfo,cv::Mat&);
		static void orientationv2(cv::Mat, std::vector<int>, linePoint&, cv::Mat&);
};

