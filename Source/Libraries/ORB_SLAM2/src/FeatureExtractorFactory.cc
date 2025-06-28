#include "FeatureExtractorFactory.h"

namespace ORB_SLAM2 {

FeatureExtractorFactory& FeatureExtractorFactory::Instance() {
    AKAZEextractor::ForceLinking();
    ORBextractor::ForceLinking();
    static FeatureExtractorFactory instance;
    return instance;
}

void FeatureExtractorFactory::Register(const std::string& name, CreatorFunc creator) {
    mRegistry[name] = creator;
}

std::shared_ptr<FeatureExtractor> FeatureExtractorFactory::Create(const std::string& name, const cv::FileNode& config, const bool init) const {
    auto it = mRegistry.find(name);
    if (it != mRegistry.end()) {
        return it->second(config, init);
    } else {
        throw std::runtime_error("[FeatureExtractorFactory] Not Registered: " + name);
    }
}
} // namespace ORB_SLAM2