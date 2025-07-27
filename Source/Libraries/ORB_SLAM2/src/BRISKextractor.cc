#include "BRISKextractor.h"
#include "FeatureExtractorFactory.h"
#include <iostream>

namespace ORB_SLAM2 {

void BRISKextractor::InfoConfigs()
{
    std::cout << "- Number of Features: " << nfeatures    << std::endl
              << "- Scale Levels: "       << nlevels      << std::endl
              << "- Scale Factor: "       << scaleFactor  << std::endl
              << "- Threshold: "          << threshold    << std::endl
              << "- Num of Octaves: "     << nOctaves     << std::endl
              << "- Pattern Scale: "      << patternScale << std::endl;
}

BRISKextractor::BRISKextractor(int nfeatures_, float scaleFactor_, int nlevels_,
                               int iniThFAST_, int minThFAST_)
    : FeatureExtractor(nfeatures_, scaleFactor_, nlevels_,
                       iniThFAST_, minThFAST_), threshold(30), nOctaves(8), patternScale(1.f)
{
    mpBRISK = cv::BRISK::create(threshold, nOctaves, patternScale);
}

BRISKextractor::BRISKextractor(const cv::FileNode& config, bool init)
    : FeatureExtractor(config, init)
{
    threshold    = config["threshold"].empty()    ? 50   : (int)config["threshold"];
    nOctaves     = config["nOctaves"].empty()     ? 8    : (int)config["nOctaves"];
    patternScale = config["patternScale"].empty() ? 1.f  : (float)config["patternScale"];

    mpBRISK = cv::BRISK::create(threshold, nOctaves, patternScale);
}


void BRISKextractor::operator()(cv::InputArray             image,
                                cv::InputArray             mask,
                                std::vector<cv::KeyPoint>& keypoints,
                                cv::OutputArray            descriptors)
{
    cv::Mat im = image.getMat();
    FeatureExtractor::ComputePyramid(im);

    cv::Mat raw;
    mpBRISK->detectAndCompute(image, FeatureExtractor::GetEdgedMask(EDGE_THRESHOLD, image, mask), keypoints, raw, /*useProvidedKeypoints=*/false);

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

        // std::cout << "[BRISK] Keypoint Cap Reached." << std::endl;
    }

    if (raw.empty()) return;

    const int PAD = ((raw.cols + 7) & ~7);
    cv::Mat aligned(raw.rows, PAD, CV_8U, cv::Scalar(0));
    raw.copyTo(aligned.colRange(0, raw.cols));
    aligned.copyTo(descriptors);
}

void BRISKextractor::ForceLinking() {}

} // namespace ORB_SLAM2

namespace {
struct BRISKRegister {
    BRISKRegister() {
        std::cout << "Registering BRISKextractor..." << std::endl;
        ORB_SLAM2::FeatureExtractorFactory::Instance().Register(
            "BRISK",
            [](const cv::FileNode& config, const bool init) {
                return new ORB_SLAM2::BRISKextractor(config, init);
            });
    }
};
BRISKRegister BriskRegisterInstance;
} // anonymous namespace
