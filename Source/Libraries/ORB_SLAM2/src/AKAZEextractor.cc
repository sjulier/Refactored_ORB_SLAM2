#include "AKAZEextractor.h"
#include <iostream>

namespace ORB_SLAM2 {

    AKAZEextractor::AKAZEextractor(int nfeatures)
        : ORBextractor(nfeatures, 1.2f, 8, 20, 7)
        , nfeatures_(nfeatures)
    {
        mpAKAZE = cv::AKAZE::create(
            cv::AKAZE::DESCRIPTOR_MLDB,
            0,          // full length
            3,          // descriptor_channels
            1e-4f,      // threshold
            4,          // num octaves
            4,          // num octave layers
            cv::KAZE::DIFF_PM_G2);
    }

    void AKAZEextractor::operator()(cv::InputArray             image,
                                    cv::InputArray             mask,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    cv::OutputArray            descriptors)
    {
        cv::Mat raw;
        mpAKAZE->detectAndCompute(image, mask, keypoints, raw, false);

        if(static_cast<int>(keypoints.size()) > nfeatures_)
        {
            cv::KeyPointsFilter::retainBest(keypoints, nfeatures_);
            raw = raw.rowRange(0, nfeatures_).clone();
        }

        const int PAD = 64;
        cv::Mat aligned(raw.rows, PAD, CV_8U, cv::Scalar(0));
        raw.copyTo(aligned.colRange(0, raw.cols));
        aligned.copyTo(descriptors);

        std::cout << "[AKAZEextractor] Keypoints: "
                  << keypoints.size()
                  << ", Descriptors shape: "
                  << descriptors.getMat().rows << "x"
                  << descriptors.getMat().cols << std::endl;
    }

} // namespace ORB_SLAM2


