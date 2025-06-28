#ifndef FEATURE_EXTRACTOR_FACTORY_H
#define FEATURE_EXTRACTOR_FACTORY_H

#include <string>
#include <functional>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include "FeatureExtractor.h"

#include "ORBextractor.h"
#include "AKAZEextractor.h"

namespace ORB_SLAM2 {

class FeatureExtractorFactory {
public:
    using CreatorFunc = std::function<FeatureExtractor *(const cv::FileNode&, const bool init)>;

    static FeatureExtractorFactory& Instance();

    void Register(const std::string& name, CreatorFunc creator);

    FeatureExtractor *Create(const std::string& name, const cv::FileNode& config, const bool init) const;

private:
    std::map<std::string, CreatorFunc> mRegistry;
};

} // namespace ORB_SLAM2

#endif
