
class RiptideVision{

	public:
		struct linePoint{
		  cv::Point top;
		  cv::Point bot;
		};
		static cv::Mat seperateColors(cv::Mat, std::vector<int>);
		static void colorAverage(cv::Mat, std::vector<int>, cv::Point&);
		static void orientation(cv::Mat, std::vector<int>, cv::Point, linePoint&);
};

