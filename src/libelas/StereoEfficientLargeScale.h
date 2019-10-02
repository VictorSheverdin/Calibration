#pragma once

#include <opencv2/opencv.hpp>
#include "elas.h"

using namespace cv;


class StereoEfficientLargeScale
{
protected:
	//Elas::parameters param(Elas::MIDDLEBURY);

	int minDisparity;
	int disparityRange;
public:
    Elas elas;
    StereoEfficientLargeScale();
    void operator()(const cv::Mat& leftim, const cv::Mat& rightim, cv::Mat& leftdisp, cv::Mat& rightdisp, int border);
    void operator()(const cv::Mat& leftim, const cv::Mat& rightim, cv::Mat& leftdisp, int border);
//	void StereoEfficientLargeScale::check(Mat& leftim, Mat& rightim, Mat& disp, StereoEval& eval);
};
