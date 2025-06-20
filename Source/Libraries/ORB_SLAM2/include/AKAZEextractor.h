#ifndef AKAZEEXTRACTOR_H
#define AKAZEEXTRACTOR_H

#include "AbstractExtractor.h"
#include <opencv2/features2d.hpp>

namespace ORB_SLAM2 {

    class AKAZEextractor : public AbstractExtractor
    {
    public:
        explicit AKAZEextractor(int nfeatures = 1000);

        void operator()(cv::InputArray                image,
                        cv::InputArray                mask,
                        std::vector<cv::KeyPoint>&    keypoints,
                        cv::OutputArray               descriptors)  override;

        int  DescriptorBytes() const override { return 64; }
        std::string Name()     const override { return "AKAZE"; }

        int GetLevels()          const override { return 1; }
        float GetScaleFactor()   const override { return 1.0f; }
        const std::vector<float>& GetScaleFactors()             const override { return mvScaleFactor_; }
        const std::vector<float>& GetInverseScaleFactors()      const override { return mvInvScaleFactor_; }
        const std::vector<float>& GetScaleSigmaSquares()        const override { return mvLevelSigma2_; }
        const std::vector<float>& GetInverseScaleSigmaSquares() const override { return mvInvLevelSigma2_; }
        const std::vector<cv::Mat>& GetImagePyramid()           const override { return mvImagePyramid_; }

    private:
        cv::Ptr<cv::AKAZE> mpAKAZE;
        int nfeatures_;

        std::vector<float> mvScaleFactor_{1,1.0f},
                           mvInvScaleFactor_{1,1.0f},
                           mvLevelSigma2_{1,1.0f},
                           mvInvLevelSigma2_{1,1.0f};
        std::vector<cv::Mat> mvImagePyramid_{1};
    };

} // namespace ORB_SLAM2

#endif //AKAZEEXTRACTOR_H
