#ifndef SIFTEXTRACTOR_H
#define SIFTEXTRACTOR_H

#include "FeatureExtractor.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

namespace ORB_SLAM2 {

class SIFTextractor : public FeatureExtractor {
    public:
        SIFTextractor(int nfeatures, float scaleFactor, int nlevels,
                      int iniThFAST, int minThFAST);

        SIFTextractor(const cv::FileNode& config, bool init = false);

        void operator()(cv::InputArray image, cv::InputArray mask,
                        std::vector<cv::KeyPoint>& keypoints,
                        cv::OutputArray descriptors) override;

        void InfoConfigs() override;
        static void ForceLinking();

    private:
        cv::Ptr<cv::SIFT> mpSIFT;
        int edgeThreshold;
        int nOctaves;
        float contrastThreshold;
        float sigma;
    };

} // namespace ORB_SLAM2

#endif //SIFTEXTRACTOR_H
