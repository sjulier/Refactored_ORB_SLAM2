#ifndef AKAZEEXTRACTOR_H
#define AKAZEEXTRACTOR_H

#include "FeatureExtractor.h"
#include <opencv2/features2d.hpp>

namespace ORB_SLAM2 {

class AKAZEextractor : public FeatureExtractor {
   public:
      AKAZEextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);

      void operator()(cv::InputArray image,
                 cv::InputArray mask,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::OutputArray descriptors) override;

   private:
      cv::Ptr<cv::AKAZE> mpAKAZE;
};

} // namespace ORB_SLAM2

#endif //AKAZEEXTRACTOR_H
