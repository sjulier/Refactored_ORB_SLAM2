#include "KAZEextractor.h"
#include "FeatureExtractorFactory.h"
#include <iostream>

namespace ORB_SLAM2 {

void KAZEextractor::InfoConfigs() {
  std::cout << "- Number of Features : " << nfeatures     << std::endl;
  std::cout << "- Scale Levels       : " << nlevels       << std::endl;
  std::cout << "- Scale Factor       : " << scaleFactor   << std::endl;
  std::cout << "- Threshold          : " << threshold     << std::endl;
  std::cout << "- nOctaves           : " << nOctaves      << std::endl;
  std::cout << "- nOctaveLayers      : " << nOctaveLayers << std::endl;
  std::cout << "- Extended (128‑dim) : " << extended      << std::endl;
  std::cout << "- Upright            : " << upright       << std::endl;
}

KAZEextractor::KAZEextractor(int nfeatures_, float scaleFactor_, int nlevels_,
                             int iniThFAST_,  int minThFAST_)
  : FeatureExtractor(nfeatures_, scaleFactor_, nlevels_, iniThFAST_, minThFAST_)
{
  mpKAZE = cv::KAZE::create(
      extended, upright, threshold,
      nOctaves, nOctaveLayers, cv::KAZE::DIFF_PM_G2);
}

KAZEextractor::KAZEextractor(const cv::FileNode& cfg, bool init)
  : FeatureExtractor(cfg, init)
{
  threshold     = cfg["threshold"].empty()      ? 1e-3f : (float)cfg["threshold"];
  nOctaves      = cfg["nOctaves"].empty()       ? 4     : (int)cfg["nOctaves"];
  nOctaveLayers = cfg["nOctaveLayers"].empty()  ? 4     : (int)cfg["nOctaveLayers"];
  extended      = cfg["extended"].empty()       ? false : (int)cfg["extended"] != 0;
  upright       = cfg["upright"].empty()        ? false : (int)cfg["upright"]  != 0;

  mpKAZE = cv::KAZE::create(
      extended, upright, threshold,
      nOctaves, nOctaveLayers, cv::KAZE::DIFF_PM_G2);
}

void KAZEextractor::operator()(cv::InputArray             image,
                               cv::InputArray             mask,
                               std::vector<cv::KeyPoint>& keypoints,
                               cv::OutputArray            descriptors)
{
  cv::Mat im = image.getMat();
  FeatureExtractor::ComputePyramid(im);

  cv::Mat raw;
  mpKAZE->detectAndCompute(image, FeatureExtractor::GetEdgedMask(EDGE_THRESHOLD, image, mask), keypoints, raw, false);

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

    // std::cout << "[KAZE] Keypoint Cap Reached." << std::endl;
  }

  raw.copyTo(descriptors);

}

void KAZEextractor::ForceLinking() {}

} // namespace ORB_SLAM2

namespace {
struct KAZERegister {
  KAZERegister(){
    std::cout << "Registering KAZEextractor..." << std::endl;
    ORB_SLAM2::FeatureExtractorFactory::Instance().Register("KAZE",
        [](const cv::FileNode& cfg, const bool init){
            return new ORB_SLAM2::KAZEextractor(cfg, init);
        });
  }
};
KAZERegister KAZERegisterInstance;
}
