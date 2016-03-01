
class RiptideVision{

	public:
		struct colorPoint{
			int x;
			int y;
		};
		struct linePoint{
		  colorPoint top;
		  colorPoint bot;
		};
		static cv::Mat seperateColors(cv::Mat, std::vector<int>);
		static void colorAverage(cv::Mat, std::vector<int>, colorPoint&);
		static void orientation(cv::Mat, std::vector<int>, colorPoint, linePoint&);
};

