#include "AKAZEextractor.h"
#include "FeatureExtractorFactory.h"
#include <iostream>

namespace ORB_SLAM2 {

    void AKAZEextractor::InfoConfigs() {
      std::cout << "- Number of Features: " << nfeatures << std::endl;
      std::cout << "- Scale Levels: " << nlevels << std::endl;
      std::cout << "- Scale Factor: " << scaleFactor << std::endl;
      std::cout << "- Threshold: " << threshold << std::endl;
      std::cout << "- Num of Octaves: " << nOctaves << std::endl;
      std::cout << "- Num of Octave Layers: " << nOctaveLayers << std::endl;
    }

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

    AKAZEextractor::AKAZEextractor(const cv::FileNode& config, bool init)
      : FeatureExtractor(config, init) {

        threshold     = config["threshold"].empty()      ? 1e-3f : (float)config["threshold"];
        nOctaves        = config["nOctaves"].empty()       ? 4     : (int)config["nOctaves"];
        nOctaveLayers   = config["nOctaveLayers"].empty()  ? 4     : (int)config["nOctaveLayers"];

        mpAKAZE = cv::AKAZE::create(
            cv::AKAZE::DESCRIPTOR_MLDB,
            0,          // full length
            3,          // descriptor_channels
            threshold,      // threshold
            nOctaves,       // num octaves
            nOctaveLayers,  // num octave layers
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

    }

    void AKAZEextractor::ForceLinking() {}

} // namespace ORB_SLAM2

namespace {

    struct AKAZERegister {
        AKAZERegister() {
            std::cout << "Registering AKAZEextractor..." << std::endl;
            ORB_SLAM2::FeatureExtractorFactory::Instance().Register("AKAZE",
                [](const cv::FileNode& config, const bool init) {
                    return new ORB_SLAM2::AKAZEextractor(config, init);
                });
        }
    };
    AKAZERegister AkazeRegisterInstance;
}


