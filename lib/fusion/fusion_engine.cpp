#include "fusion/fusion_engine.h"

#include <opencv2/imgproc.hpp>

void FusionEngine::fuse(const std::vector<cv::Mat> &lapA,
                        const std::vector<cv::Mat> &lapB,
                        std::vector<cv::Mat> &lapFused)
{
    const int levels = (int)lapA.size();
    lapFused.resize(levels);

    /* Matrix that adds the 3 colour channels: |B|+|G|+|R| */
    const cv::Matx13f sumChannels(1.f, 1.f, 1.f);

    /* ---- detail bands: "choose max" ---- */
    for (int i = 0; i < levels - 1; i++)
    {
        /* Activity of each pixel = sum of absolute coefficients over the
         * colour channels, slightly blurred so one noisy pixel does not
         * flip the decision on its own.                                 */
        absTmp_ = cv::abs(lapA[i]);
        cv::transform(absTmp_, magA_, sumChannels);
        cv::blur(magA_, magA_, cv::Size(3, 3));

        absTmp_ = cv::abs(lapB[i]);
        cv::transform(absTmp_, magB_, sumChannels);
        cv::blur(magB_, magB_, cv::Size(3, 3));

        /* mask = 255 where camera A has more detail than camera B */
        cv::compare(magA_, magB_, mask_, cv::CMP_GE);

        /* Start from B, then overwrite with A wherever A is stronger */
        lapB[i].copyTo(lapFused[i]);
        lapA[i].copyTo(lapFused[i], mask_);
    }

    /* ---- base band: average ---- */
    cv::addWeighted(lapA[levels - 1], 0.5, lapB[levels - 1], 0.5, 0.0, lapFused[levels - 1]);
}
