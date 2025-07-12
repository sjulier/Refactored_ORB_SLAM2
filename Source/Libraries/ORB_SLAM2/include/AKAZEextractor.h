#ifndef AKAZEEXTRACTOR_H
#define AKAZEEXTRACTOR_H

#include "FeatureExtractor.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

namespace ORB_SLAM2 {

class AKAZEextractor : public FeatureExtractor {
   public:
      AKAZEextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);

      AKAZEextractor(const cv::FileNode& config, bool init = false);

      void operator()(cv::InputArray image,
                 cv::InputArray mask,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::OutputArray descriptors) override;

	  void InfoConfigs() override;

      static void ForceLinking();

   private:
      cv::Ptr<cv::AKAZE> mpAKAZE;
      float threshold;
      int nOctaves;
      int nOctaveLayers;
};

} // namespace ORB_SLAM2

#endif //AKAZEEXTRACTOR_H
