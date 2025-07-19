#ifndef BRISKEXTRACTOR_H
#define BRISKEXTRACTOR_H

#include "FeatureExtractor.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

namespace ORB_SLAM2 {

    class BRISKextractor : public FeatureExtractor {
    public:
        BRISKextractor(int nfeatures, float scaleFactor, int nlevels,
                       int iniThFAST, int minThFAST);

        BRISKextractor(const cv::FileNode& config, bool init = false);

        void operator()(cv::InputArray image, cv::InputArray mask,
                        std::vector<cv::KeyPoint>& keypoints,
                        cv::OutputArray descriptors) override;

        void InfoConfigs() override;

        static void ForceLinking();

    private:
        cv::Ptr<cv::BRISK> mpBRISK;
        int   thresh;
        int   octaves;
        float patternScale;
    };

} // namespace ORB_SLAM2
#endif // BRISKEXTRACTOR_H

