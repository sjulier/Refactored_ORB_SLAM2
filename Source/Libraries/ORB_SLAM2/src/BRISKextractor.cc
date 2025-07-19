#include "BRISKextractor.h"
#include "FeatureExtractorFactory.h"
#include <iostream>

namespace ORB_SLAM2 {

void BRISKextractor::InfoConfigs()
{
    std::cout << "- Number of Features: " << nfeatures    << std::endl
              << "- Scale Levels: "       << nlevels      << std::endl
              << "- Scale Factor: "       << scaleFactor  << std::endl
              << "- thresh: "             << thresh       << std::endl
              << "- octaves: "            << octaves      << std::endl
              << "- patternScale: "       << patternScale << std::endl;
}

BRISKextractor::BRISKextractor(int nfeatures_, float scaleFactor_, int nlevels_,
                               int iniThFAST_, int minThFAST_)
    : FeatureExtractor(nfeatures_, scaleFactor_, nlevels_,
                       iniThFAST_, minThFAST_),
      thresh(30), octaves(8), patternScale(1.f)
{
    mpBRISK = cv::BRISK::create(thresh, octaves, patternScale);
}

BRISKextractor::BRISKextractor(const cv::FileNode& config, bool init)
    : FeatureExtractor(config, init)
{
    thresh       = config["thresh"].empty()       ? 50   : (int)config["thresh"];
    octaves      = config["octaves"].empty()      ? 8    : (int)config["octaves"];
    patternScale = config["patternScale"].empty() ? 1.f  : (float)config["patternScale"];

    mpBRISK = cv::BRISK::create(thresh, octaves, patternScale);
}


void BRISKextractor::operator()(cv::InputArray             image,
                                cv::InputArray             mask,
                                std::vector<cv::KeyPoint>& keypoints,
                                cv::OutputArray            descriptors)
{
    cv::Mat im = image.getMat();
    FeatureExtractor::ComputePyramid(im);

    cv::Mat raw;
    mpBRISK->detectAndCompute(image, mask, keypoints, raw, /*useProvidedKeypoints=*/false);


 	if(static_cast<int>(keypoints.size()) > nfeatures) {
            cv::KeyPointsFilter::retainBest(keypoints, nfeatures);
            raw = raw.rowRange(0, nfeatures).clone();
    }

    const int PAD = 64;
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
