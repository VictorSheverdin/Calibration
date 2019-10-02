#include <opencv2/opencv.hpp>
#include <vector>

class Frame
{
public:
    Frame();

private:
    void initialize();

    std::vector<cv::Point2f> m_featurePoints;
};
