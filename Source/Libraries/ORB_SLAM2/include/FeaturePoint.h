#ifndef FEATUREPOINT_H
#define FEATUREPOINT_H

#include <vector>

//#include "DBoW2/BowVector.h"
//#include "DBoW2/FeatureVector.h"

#include <fbow.h>

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2 {

class MapPoint;

class FeaturePoint
{
public:
  
  // Feature type
  int type;

  // Number of Keypoints
  int N;

  // Parms
  std::vector<cv::KeyPoint> mvKeys;
  std::vector<cv::KeyPoint> mvKeysRight;
  std::vector<cv::KeyPoint> mvKeysUn;

  std::vector<float> mvuRight;
  std::vector<float> mvDepth;
  
  cv::Mat mDescriptors; 
  cv::Mat mDescriptorsRight;

  std::vector<MapPoint *> mvpMapPoints;
  std::vector<bool> mvbOutlier;

  std::vector<std::vector<std::vector<std::size_t>>> mGrid;

  // Bag of Words std::vector structures.
  // DBoW2::BowVector mBowVec;
  // DBoW2::FeatureVector mFeatVec;
  fbow::fBow mBowVec;
  fbow::fBow2 mFeatVec;
};
    
}
#endif