#ifndef FEATURE_EXTRACTOR_FACTORY_H
#define FEATURE_EXTRACTOR_FACTORY_H

#include "FeatureExtractor.h"

namespace ORB_SLAM2 {

// ORB:0, AKAZE:1
FeatureExtractor *createFeatureExtractor(int nfeatures, float scaleFactor,
                                         int nlevels, int iniThFAST,
                                         int minThFAST, int flag);

}
#endif
