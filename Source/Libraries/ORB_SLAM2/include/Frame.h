/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FRAME_H
#define FRAME_H

#include <vector>

// #include "DBoW2/BowVector.h"
// #include "DBoW2/FeatureVector.h"
#include <fbow.h>

#include "FeatureExtractor.h"
#include "FeaturePoint.h"
#include "KeyFrame.h"
#include "MapPoint.h"
// #include "ORBVocabulary.h"
#include "FbowVocabulary.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2 {
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

// #define FRAME_GRID_ROWS 14
// #define FRAME_GRID_COLS 16

class MapPoint;
class KeyFrame;

class Frame {
public:

  Frame(int Ntype);
  
  // Copy constructor.
  Frame(const Frame &frame);

  // Constructor for stereo cameras.
  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
        std::vector<FeatureExtractor *> extractorLeft, std::vector<FeatureExtractor *> extractorRight,
        std::vector<FbowVocabulary *> voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth, int Ntype);

  // Constructor for RGB-D cameras.
  Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp,
        std::vector<FeatureExtractor *> extractor,
        std::vector<FbowVocabulary *> voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth, int Ntype);

  // Constructor for Monocular cameras.
  Frame(const cv::Mat &imGray, const double &timeStamp,
        std::vector<FeatureExtractor *> extractor,
        std::vector<FbowVocabulary *> voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth, int Ntype);

  // Extract features, Ftype: ORB(0), GCN(1), imageFlag: left image (0), right image (1).
  void ExtractFeatures(const int Ftype, int imageFlag, const cv::Mat &im);

  // Compute Bag of Words representation.
  void ComputeBoW(const int Ftype);

  // Set the camera pose.
  void SetPose(cv::Mat Tcw);

  // Computes rotation, translation and camera center matrices from the camera pose.
  void UpdatePoseMatrices();

  // Returns the camera center.
  inline cv::Mat GetCameraCenter() { return mOw.clone(); }

  // Returns inverse of rotation
  inline cv::Mat GetRotationInverse() { return mRwc.clone(); }

  // Check if a MapPoint is in the frustum of the camera and fill variables of the MapPoint to be used by the tracking
  bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

  // Compute the cell of a keypoint (return false if outside the grid)
  bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
  
  std::vector<size_t> GetFeaturesInArea(const int Ftype, const float &x, const float &y, const float &r, const int minLevel = -1,
                                        const int maxLevel = -1) const;

  // Search a match for each keypoint in the left image to a keypoint in the right image. If there is a match, depth is computed and the right
  // coordinate associated to the left keypoint is stored.
  void ComputeStereoMatches(const int Ftype);

  // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
  void ComputeStereoFromRGBD(const cv::Mat &imDepth, const int Ftype);
  void ComputeStereoFromRGBD(const cv::Mat &imDepth, std::vector<float> &uRight, std::vector<float> &Depth, const int &refN, 
                             const std::vector<cv::KeyPoint> &Keys, const std::vector<cv::KeyPoint> &KeysUn);

  // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
  cv::Mat UnprojectStereo(const int &i, const int Ftype);
  cv::Mat UnprojectStereo(const int &i, const std::vector<float> &Depth, const std::vector<cv::KeyPoint> &KeysUn);

public:
  int Ntype;

  // Vocabulary vector used for relocalization
  std::vector<FbowVocabulary *> mpVocabulary;

  // Feature extractor. The right is used only in the stereo case.
  std::vector<FeatureExtractor *> mpFeatureExtractorLeft;
  std::vector<FeatureExtractor *> mpFeatureExtractorRight;

  // Frame timestamp.
  double mTimeStamp;

  // Calibration matrix and OpenCV distortion parameters.
  cv::Mat mK;
  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;
  cv::Mat mDistCoef;

  // Stereo baseline multiplied by fx.
  float mbf;

  // Stereo baseline in meters.
  float mb;

  // Threshold close/far points. Close points are inserted from 1 view. Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

  // Feature data used to store feature points
  std::vector<FeaturePoint> Channels;

  // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;

  // Camera pose.
  cv::Mat mTcw;

  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Reference Keyframe.
  KeyFrame *mpReferenceKF;

  // Scale pyramid info.
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  std::vector<float> mvScaleFactors;
  std::vector<float> mvInvScaleFactors;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;

  // Undistorted Image Bounds (computed once).
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static bool mbInitialComputations;

private:
  // Undistort keypoints given OpenCV distortion parameters. Only for the RGB-D case. Stereo must be already rectified! (called in the constructor).
  void UndistortKeyPoints(const int Ftype);
  void UndistortKeyPoints(const std::vector<cv::KeyPoint> &Keys, std::vector<cv::KeyPoint> &KeysUn, const int &refN);

  // Computes image bounds for the undistorted image (called in the constructor).
  void ComputeImageBounds(const cv::Mat &imLeft);

  // Assign keypoints to the grid for speed up feature matching (called in the constructor).
  void AssignFeaturesToGrid(const int Ftype);
  void AssignFeaturesToGrid(const int &refN, const std::vector<cv::KeyPoint> &KeysUn, std::vector<std::vector<std::vector<std::size_t>>> &Grid);

  // compute features and assign to grids
  void ComputeFeaturesRGBD(const int Ftype, const cv::Mat &imGray, const cv::Mat &imDepth);
  void ComputeFeaturesStereo(const int Ftype, const cv::Mat &imLeft, const cv::Mat &imRight);
  void ComputeFeaturesMono(const int Ftype, const cv::Mat &imGray); 
  
  // Rotation, translation and camera center
  cv::Mat mRcw;
  cv::Mat mtcw;
  cv::Mat mRwc;
  cv::Mat mOw; //==mtwc
};

} // namespace ORB_SLAM2

#endif // FRAME_H
