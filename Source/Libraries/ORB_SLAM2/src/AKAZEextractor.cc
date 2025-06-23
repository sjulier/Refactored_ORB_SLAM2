#include "AKAZEextractor.h"
#include <iostream>

namespace ORB_SLAM2 {

    AKAZEextractor::AKAZEextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST)
      : FeatureExtractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST) {
        mpAKAZE = cv::AKAZE::create(
            cv::AKAZE::DESCRIPTOR_MLDB,
            0,          // full length
            3,          // descriptor_channels
            1e-3f,      // threshold
            4,          // num octaves
            4,          // num octave layers
            cv::KAZE::DIFF_PM_G2);
    }

    void AKAZEextractor::operator()(cv::InputArray             image,
                                    cv::InputArray             mask,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    cv::OutputArray            descriptors)
    {
        mvImagePyramid[0] = image.getMat();

        cv::Mat raw;
        mpAKAZE->detectAndCompute(image, mask, keypoints, raw, false);

        for(auto& kp : keypoints) kp.octave = 0;

        if(static_cast<int>(keypoints.size()) > nfeatures)
        {
            cv::KeyPointsFilter::retainBest(keypoints, nfeatures);
            raw = raw.rowRange(0, nfeatures).clone();
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


