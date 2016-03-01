
class RiptideVision{

	public:
		struct colorPoint{
			int x;
			int y;
		};
		static cv::Mat seperateColors(cv::Mat, std::vector<int>);
		static void colorAverage(cv::Mat, std::vector<int>, colorPoint);
};

