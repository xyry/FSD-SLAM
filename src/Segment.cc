/*
 *--------------------------------------------------------------------------------------------------
 * DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments
　*　Author(s):
 * Chao Yu, Zuxin Liu, Xinjun Liu, Fugui Xie, Yi Yang, Qi Wei, Fei Qiao qiaofei@mail.tsinghua.edu.cn
 * Created by Yu Chao@2018.12.03
 * --------------------------------------------------------------------------------------------------
 * DS-SLAM is a optimized SLAM system based on the famous ORB-SLAM2. If you haven't learn ORB_SLAM2 code, 
 * you'd better to be familiar with ORB_SLAM2 project first. Compared to ORB_SLAM2, 
 * we add anther two threads including semantic segmentation thread and densemap creation thread. 
 * You should pay attention to Frame.cc, ORBmatcher.cc, Pointcloudmapping.cc and Segment.cc.
 * 
 *　@article{murORB2,
 *　title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
　*　author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
　* journal={IEEE Transactions on Robotics},
　*　volume={33},
　* number={5},
　* pages={1255--1262},
　* doi = {10.1109/TRO.2017.2705103},
　* year={2017}
 *　}
 * --------------------------------------------------------------------------------------------------
 * Copyright (C) 2018, iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/) and 
 * Advanced Mechanism and Roboticized Equipment Lab. All rights reserved.
 *
 * Licensed under the GPLv3 License;
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * https://github.com/ivipsourcecode/DS-SLAM/blob/master/LICENSE
 *--------------------------------------------------------------------------------------------------
 */

#include "Segment.h"
#include "Tracking.h"
#include "Camera.h"
#include <fstream>
#define SKIP_NUMBER 1
using namespace std;

namespace ORB_SLAM2
{
Segment::Segment(const string &pascal_prototxt, const string &detection_config, const string &pascal_caffemodel, const string &detection_weights, const string &pascal_png,float DetectionRatio):mbFinishRequested(false),mSkipIndex(SKIP_NUMBER),mSegmentTime(0),imgIndex(0)
{

    segment_prototxt_file = pascal_prototxt;
    segment_model_file = pascal_caffemodel;
    segment_LUT_file = pascal_png;
    mDetectionRatio=DetectionRatio;
    detection_config_file = detection_config;
    detection_weights_file = detection_weights;
    label_colours = cv::imread(segment_LUT_file,1);
    cv::cvtColor(label_colours, label_colours, CV_RGB2BGR);
    mImgSegmentLatest=cv::Mat(Camera::height,Camera::width,CV_8UC1);
    mbNewImgFlag=false;


}

void Segment::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

bool Segment::isNewImgArrived()
{
    unique_lock<mutex> lock(mMutexGetNewImg);
    if(mbNewImgFlag)
    {
        mbNewImgFlag=false;
        return true;
    }
    else
    return false;
}

int num_segmentation=0;
int num_object_dection=0;

void Segment::Run()
{
    classifier=new Classifier(segment_prototxt_file, segment_model_file);
    detector = new Detector(detection_config_file, detection_weights_file);
    num_segmentation=0;
    num_object_detection=0;
    time_segmentation=0;
    time_object_detection=0;
    cout << "Load model ..."<<endl;
    cout << "mDetectionRatio: "<<mDetectionRatio<<endl;
    while(1)
    {
        
        usleep(1);
        if(!isNewImgArrived())
        continue;

        cout << "Wait for new RGB img time =" << endl;
        if(mSkipIndex==SKIP_NUMBER)
        {
	    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
	    mDetect_result = detector->detect(mImg);
	    std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();	    
	    int people_area = 0;
	    
	    for(auto detect_temp:mDetect_result)
	    {
	      if(detect_temp.obj_id == 0)
	      {
		people_area+=detect_temp.w*detect_temp.h;
	      }
	    }
	    cout<<"people_area = "<<people_area<<endl;
	    cout<<"people ratio = "<<1.0*people_area/(Camera::width*Camera::height)<<endl;
	   if(1.0*people_area/(Camera::width*Camera::height) > mDetectionRatio)
	   { 
            num_segmentation++;
        //   cout<<"Segmentation"<<num_segmentation<<endl;
	      mDetect_segment_flag = 1;
	      std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
              // Recognise by Semantin segmentation
              mImgSegment=classifier->Predict(mImg, label_colours);
              mImgSegment_color = mImgSegment.clone();
              cv::cvtColor(mImgSegment,mImgSegment_color, CV_GRAY2BGR);

              LUT(mImgSegment_color, label_colours, mImgSegment_color_final);
              cv::resize(mImgSegment, mImgSegment, cv::Size(Camera::width,Camera::height) );
              cv::resize(mImgSegment_color_final, mImgSegment_color_final, cv::Size(Camera::width,Camera::height) );

              std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
              mSegmentTime+=std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count();
              time_segmentation+=std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count();
              mSkipIndex=0;
              imgIndex++;
	    }
	    else
	    {
            num_object_detection++;
            // cout<<"Object Detection:"<<num_object_dection<<endl;
	      mDetect_segment_flag = 0;
	      mSegmentTime+=std::chrono::duration_cast<std::chrono::duration<double> >(t6 - t5).count();
	      time_object_detection+=std::chrono::duration_cast<std::chrono::duration<double> >(t6 - t5).count();
	      mSkipIndex=0;
              imgIndex++;
	    }
	      
       }
       mSkipIndex++;
       ProduceImgSegment();
       if(CheckFinish())
       {
          break;
       }

    }
}

bool Segment::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}
  
void Segment::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested=true;
}

void Segment::ProduceImgSegment()
{
    std::unique_lock <std::mutex> lock(mMutexNewImgSegment);
    
    if(mDetect_segment_flag == 1)
    {
      mImgTemp=mImgSegmentLatest;
      mImgSegmentLatest=mImgSegment;
      mImgSegment=mImgTemp;
    }
    else
    {
      mDetect_result_temp = mDetect_result_latest;
      mDetect_result_latest = mDetect_result;
      mDetect_result = mDetect_result_temp;
    }
    
    mpTracker->mbNewSegImgFlag=true;
}

}
