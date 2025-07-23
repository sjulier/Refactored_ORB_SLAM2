#ifndef KAZEEXTRACTOR_H
#define KAZEEXTRACTOR_H

#pragma once
#include "FeatureExtractor.h"
#include <opencv2/features2d.hpp>

namespace ORB_SLAM2 {

class KAZEextractor : public FeatureExtractor {
 public:

  KAZEextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);
  KAZEextractor(const cv::FileNode& config, bool init = false);

  void operator()(cv::InputArray image,
                  cv::InputArray mask,
                  std::vector<cv::KeyPoint>& keypoints,
                  cv::OutputArray descriptors) override;

  void InfoConfigs() override;

  static void ForceLinking();

 private:
  cv::Ptr<cv::KAZE> mpKAZE;

  float threshold{1e-3f};
  int   nOctaves{4};
  int   nOctaveLayers{4};
  bool  extended{false};  // 64‑dim (false) / 128‑dim (true)
  bool  upright{false};
};

}  // namespace ORB_SLAM2


#endif //KAZEEXTRACTOR_H
