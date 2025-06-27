#include "FeatureExtractorFactory.h"

#include "ORBextractor.h"
#include "AKAZEextractor.h"

//#ifdef USE_GCN
//#include "GCNextractor.h"
//#endif

namespace ORB_SLAM2 {

FeatureExtractor *createFeatureExtractor(int nfeatures, float scaleFactor,
                                         int nlevels, int iniThFAST,
                                         int minThFAST, int flag) {
//#ifdef USE_GCN
//  if (flag == 1) {
//    return new GCNextractor(nfeatures, scaleFactor, nlevels, iniThFAST,
//                            minThFAST);
//  }
//#endif
    if (flag == 1) {
        return new AKAZEextractor(4000, scaleFactor, nlevels, iniThFAST, minThFAST);
    }
  //return new ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
    return new ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
}
} // namespace ORB_SLAM2
