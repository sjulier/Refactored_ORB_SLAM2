#include "SIFTextractor.h"
#include "FeatureExtractorFactory.h"
#include <iostream>

namespace ORB_SLAM2 {

void SIFTextractor::InfoConfigs() {
    std::cout << "- Number of Features  : " << nfeatures << std::endl;
    std::cout << "- Scale Levels        : " << nlevels   << std::endl;
    std::cout << "- Scale Factor        : " << scaleFactor << std::endl;
    std::cout << "- Contrast Threshold  : " << contrastThreshold << std::endl;
    std::cout << "- Edge Threshold      : " << edgeThreshold << std::endl;
    std::cout << "- nOctaves            : " << nOctaves << std::endl;
    std::cout << "- Sigma               : " << sigma << std::endl;
}

SIFTextractor::SIFTextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST,int minThFAST)
    : FeatureExtractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST)
{
    contrastThreshold = 0.04;
    edgeThreshold     = 10;
    sigma             = 1.6;
    nOctaves          = 3;

    mpSIFT = cv::SIFT::create(nfeatures, nOctaves,
                              contrastThreshold,
                              edgeThreshold,
                              sigma);
}

SIFTextractor::SIFTextractor(const cv::FileNode& config,bool init)
    : FeatureExtractor(config,init)
{
    edgeThreshold     = config["edgeThreshold"].empty() ? 10 : (int)config["edgeThreshold"];
    contrastThreshold = config["contrastThreshold"].empty() ? 0.04: (float)config["contrastThreshold"];
    sigma             = config["sigma"].empty() ? 1.6 : (float)config["sigma"];
    nOctaves          = config["nOctaves"].empty() ? 3 : (int)config["nOctaves"];

    mpSIFT = cv::SIFT::create(nfeatures, nOctaves,
                              contrastThreshold,
                              edgeThreshold,
                              sigma);
}

void SIFTextractor::operator()(cv::InputArray image, cv::InputArray mask,
                               std::vector<cv::KeyPoint>& keypoints,
                               cv::OutputArray descriptors)
{
    cv::Mat im = image.getMat();
    FeatureExtractor::ComputePyramid(im);

    cv::Mat raw;
    mpSIFT->detectAndCompute(image, mask, keypoints, raw, false);

	for(auto& kp : keypoints) kp.octave = 0;

    std::cout << "[SIFT] detectAndCompute -> "
              << "kpts=" << keypoints.size()
              << ", rows=" << raw.rows
              << ", cols=" << raw.cols
              << ", type=" << raw.type()
              << std::endl;

    raw.copyTo(descriptors);   // SIFT descriptor: CV_32F 128 dim
}

void SIFTextractor::ForceLinking(){}

} // namespace ORB_SLAM2

namespace {

    struct SIFTRegister {
        SIFTRegister() {
            std::cout << "Registering SIFTextractor..." << std::endl;
            ORB_SLAM2::FeatureExtractorFactory::Instance().Register("SIFT",
            [](const cv::FileNode& config, const bool init){
                return new ORB_SLAM2::SIFTextractor(config, init);
            });
        }
    };
    SIFTRegister SIFTRegisterInstance;
}

