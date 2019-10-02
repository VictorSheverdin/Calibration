//#define PROFILE 1
/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/
#include "descriptor.h"
#include "triangle.h"
#include "matrix.h"

#include "StereoEfficientLargeScale.h"
using namespace std;

StereoEfficientLargeScale::StereoEfficientLargeScale()
{
}
void StereoEfficientLargeScale::operator()(const cv::Mat& leftim, const cv::Mat& rightim, cv::Mat& leftdisp, cv::Mat& rightdisp, int bd)
{
	Mat l,r;
    if(leftim.channels()==3){cvtColor(leftim,l,cv::COLOR_BGR2GRAY);cout<<"convert gray"<<endl;}
	else l=leftim;
    if(rightim.channels()==3)cvtColor(rightim,r,cv::COLOR_BGR2GRAY);
	else r=rightim;

	Mat lb,rb;
	cv::copyMakeBorder(l,lb,0,0,bd,bd,cv::BORDER_REPLICATE);
	cv::copyMakeBorder(r,rb,0,0,bd,bd,cv::BORDER_REPLICATE);

	const cv::Size imsize = lb.size();
	const int32_t dims[3] = {imsize.width,imsize.height,imsize.width}; // bytes per line = width

	cv::Mat leftdpf = cv::Mat::zeros(imsize,CV_32F);
	cv::Mat rightdpf = cv::Mat::zeros(imsize,CV_32F);
	elas.process(lb.data,rb.data,leftdpf.ptr<float>(0),rightdpf.ptr<float>(0),dims);

	Mat disp;
	Mat(leftdpf(cv::Rect(bd,0,leftim.cols,leftim.rows))).copyTo(disp);
	disp.convertTo(leftdisp,CV_16S,16);
	Mat(rightdpf(cv::Rect(bd,0,leftim.cols,leftim.rows))).copyTo(disp);

	disp.convertTo(rightdisp,CV_16S,16);
}

void StereoEfficientLargeScale::operator()(const cv::Mat& leftim, const cv::Mat& rightim, cv::Mat& leftdisp, int bd)
{
	Mat temp;
	StereoEfficientLargeScale::operator()(leftim,rightim,leftdisp,temp,bd);
}
/*
void StereoEfficientLargeScale::check(Mat& leftim, Mat& rightim, Mat& disp, StereoEval& eval)
{
	string wname = "ELAS";
	namedWindow(wname);
	int nsigma = 0;
	createTrackbar("N sigma",wname,&nsigma,1000);

	int key = 0;
	Mat disp16;
	Mat dispr16;
	Mat lim,rim;

	Stat st;
	while(key!='q')
	{
		addNoise(leftim,lim,nsigma/10.0);
		addNoise(rightim,rim,nsigma/10.0);
		
		operator()(lim,rim,disp16,dispr16,16);

		Mat disp8;
		disp16.convertTo(disp8,CV_8U,2/16.0);
		eval(disp8,1.0,true);
		st.push_back(eval.all);
		st.show();
		imshow(wname,disp);
		if(key=='r')st.clear();
		key = waitKey(1);
	}
}*/
