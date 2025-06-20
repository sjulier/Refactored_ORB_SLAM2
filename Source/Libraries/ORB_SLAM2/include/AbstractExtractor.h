
#ifndef ABSTRACTEXTRACTOR_H
#define ABSTRACTEXTRACTOR_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <string>

namespace ORB_SLAM2 {

    /**
     * Uniform Feature Abstract Base Class, all feature need implement through this.
     */
    class AbstractExtractor
    {
    public:
        virtual ~AbstractExtractor() = default;

        // ---- Core Feature ----
        virtual void operator()(cv::InputArray             image,
                                cv::InputArray             mask,
                                std::vector<cv::KeyPoint>& kps,
                                cv::OutputArray            desc) = 0;

        // ---- Meta Data ----
        virtual int  DescriptorBytes() const = 0;          // Descriptor Byte Length
        virtual std::string Name()     const = 0;          // Feature Meta Name

        // ---- Pyramid Parameters ----
        //   ( for Frame / Stereo / BA )
        virtual int  GetLevels()                          const = 0;
        virtual float GetScaleFactor()                    const = 0;
        virtual const std::vector<float>& GetScaleFactors()             const = 0;
        virtual const std::vector<float>& GetInverseScaleFactors()      const = 0;
        virtual const std::vector<float>& GetScaleSigmaSquares()        const = 0;
        virtual const std::vector<float>& GetInverseScaleSigmaSquares() const = 0;
        virtual const std::vector<cv::Mat>& GetImagePyramid()           const = 0;
    };

} // namespace ORB_SLAM2

#endif //ABSTRACTEXTRACTOR_H
