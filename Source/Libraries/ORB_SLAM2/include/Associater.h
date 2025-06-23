#ifndef ASSOCIATER_H
#define ASSOCIATER_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM2 {

class Associater {
public:
  Associater(float nnratio = 0.6, bool checkOri = true);

  // Computes the Hamming distance between two ORB descriptors
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

  // Project MapPoints tracked in last frame into the current frame and search matches. Used to track from previous frame (Tracking)
  int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono, const int Ftype);

  // Search matches between Frame keypoints and projected MapPoints. Returns number of matches Used to track the local map (Tracking)
  int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th);

  // Project MapPoints using a Similarity Transformation and search matches. Used in loop detection (Loop Closing)
  int SearchByProjection(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints, std::vector<MapPoint *> &vpMatched, int th, const int Ftype);

  // Project MapPoints seen in KeyFrame into the Frame and search matches. Used in relocalisation (Tracking)
  int SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const std::set<MapPoint *> &sAlreadyFound, const float th, const int ORBdist, const int Ftype);

  int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint *> &vpMapPointMatches, const int Ftype);

  int SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12, const int Ftype);

  int SearchByNN(Frame &CurrentFrame, const Frame &LastFrame, const int Ftype);
  
  int SearchByNN(KeyFrame *pKF, Frame &F, std::vector<MapPoint *> &vpMapPointMatches, const int Ftype);

  int SearchByNN(Frame &F, const std::vector<MapPoint *> &vpMapPoints);

  // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12] In the stereo and RGB-D case, s12=1
  int SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th, const int Ftype);

  // Matching for the Map Initialization (only used in the monocular case)
  int SearchForInitialization(const int Ftype, Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize = 10);

  int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12, std::vector<std::pair<size_t, size_t>> &vMatchedPairs, 
                             const bool bOnlyStereo, const int Ftype);

  // Project MapPoints into KeyFrame and search for duplicated MapPoints.
  int Fuse(const int Ftype, KeyFrame *pKF, const std::vector<MapPoint *> &vpMapPoints, const float th = 3.0);

  int Fuse(const int Ftype, KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints, float th, std::vector<MapPoint *> &vpReplacePoint);

public:
  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

protected:
  float mfNNratio;
  bool mbCheckOrientation;

  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

  float RadiusByViewingCos(const float &viewCos);

  bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

};

}

#endif