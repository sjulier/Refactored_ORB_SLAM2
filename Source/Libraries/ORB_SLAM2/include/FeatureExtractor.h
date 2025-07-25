
#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <list>
#include <opencv2/opencv.hpp>
#include <vector>

namespace ORB_SLAM2 {
class FeatureExtractor {
public:
  enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

  static const int PATCH_SIZE = 31;
  static const int HALF_PATCH_SIZE = 15;
  static const int EDGE_THRESHOLD = 19;

  FeatureExtractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST,
                   int minThFAST);

  // Constructor with .yaml node config as input
  FeatureExtractor(const cv::FileNode& config, bool init = false);

  virtual ~FeatureExtractor() {}

  virtual void InfoConfigs() = 0;

  virtual void operator()(cv::InputArray image, cv::InputArray mask,
                          std::vector<cv::KeyPoint> &keypoints,
                          cv::OutputArray descriptors) = 0;

  cv::Mat GetEdgedMask(int edge, cv::InputArray image, cv::InputArray mask);

  int GetLevels() const { return nlevels; }

  float GetScaleFactor() const { return scaleFactor; }

  std::vector<float> GetScaleFactors() const { return mvScaleFactor; }

  std::vector<float> GetInverseScaleFactors() const { return mvInvScaleFactor; }

  std::vector<float> GetScaleSigmaSquares() const { return mvLevelSigma2; }

  std::vector<float> GetInverseScaleSigmaSquares() const {
    return mvInvLevelSigma2;
  }

  std::vector<cv::Mat> mvImagePyramid;

protected:
  int nfeatures;
  double scaleFactor;
  int nlevels;
  int iniThFAST;
  int minThFAST;

  void InitPyramidParameters();

  void ComputePyramid(cv::Mat image);

  std::vector<int> mnFeaturesPerLevel;

  std::vector<int> umax;

  std::vector<float> mvScaleFactor;
  std::vector<float> mvInvScaleFactor;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;
};
} // namespace ORB_SLAM2

#endif
