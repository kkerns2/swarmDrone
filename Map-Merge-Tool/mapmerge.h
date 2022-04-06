#ifndef MAPMERGE_H
#define MAPMERGE_H

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

// <Referrence> http://independence-sys.net/main/?p=2632
class StitchedMap
{
public:
  // Constructor and Destructor
  StitchedMap(Mat &im1, Mat &im2, float max_distance, float matches_threshold);
  ~StitchedMap();
  
  // Represents an array of arbitrary dimensions in cv::Mat
  cv::Mat get_debug();
  cv::Mat get_stitch();
  
  // transformation matrix
  cv::Mat H;
  
  // Variable to copy the target image to
  cv::Mat image1;
  cv::Mat image2;

  // Variables for storing image feature information
  cv::Mat dscv1;
  cv::Mat dscv2;
  
  // Variable for storing feature point information
  vector<KeyPoint> kpv1;
  vector<KeyPoint> kpv2;

  // Store the keypoints of query(image1)
  vector<KeyPoint> fil1;
  // Store the keypoints of query(image2)
  vector<KeyPoint> fil2;

  // Store the keypoint coordinates of query(image1)
  vector<Point2f> coord1;
  // Store the keypoint coordinates of train(image2)
  vector<Point2f> coord2;
  
  // Variable to store matching information of feature points
  vector<DMatch>   matches;

  double rotation,transx,transy,scalex,scaley;
};

#endif // MAPMERGE_H
