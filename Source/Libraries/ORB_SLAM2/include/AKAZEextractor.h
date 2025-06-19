#ifndef AKAZEEXTRACTOR_H
#define AKAZEEXTRACTOR_H

#include "ORBextractor.h"
#include <opencv2/features2d.hpp>

namespace ORB_SLAM2 {

    class AKAZEextractor : public ORBextractor
    {
    public:
        explicit AKAZEextractor(int nfeatures = 1000);

        void operator()(cv::InputArray                image,
                        cv::InputArray                mask,
                        std::vector<cv::KeyPoint>&    keypoints,
                        cv::OutputArray               descriptors)  override;

    private:
        cv::Ptr<cv::AKAZE> mpAKAZE;
        int nfeatures_;
    };

} // namespace ORB_SLAM2

#endif //AKAZEEXTRACTOR_H
