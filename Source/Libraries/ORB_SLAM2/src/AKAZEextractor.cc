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

        threshold       = config["threshold"].empty()      ? 1e-3f : (float)config["threshold"];
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
        cv::Mat im = image.getMat();
        FeatureExtractor::ComputePyramid(im);

        cv::Mat raw;
        mpAKAZE->detectAndCompute(image, FeatureExtractor::GetEdgedMask(EDGE_THRESHOLD, image, mask), keypoints, raw, false);

        //for(auto& kp : keypoints) kp.octave = 0;

        if (raw.empty()) return;

        if (static_cast<int>(keypoints.size()) > nfeatures) {

            for (size_t i = 0; i < keypoints.size(); ++i)
                keypoints[i].class_id = static_cast<int>(i);

            cv::KeyPointsFilter::retainBest(keypoints, nfeatures);

            cv::Mat raw_sorted(static_cast<int>(keypoints.size()), raw.cols, raw.type());
            for (size_t i = 0; i < keypoints.size(); ++i) {
                int oldIdx = keypoints[i].class_id;
                raw.row(oldIdx).copyTo(raw_sorted.row(static_cast<int>(i)));
            }
            raw = raw_sorted;

            // std::cout << "[AKAZE] Keypoint Cap Reached." << std::endl;
        }

        const int PAD = ((raw.cols + 7) & ~7);
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


