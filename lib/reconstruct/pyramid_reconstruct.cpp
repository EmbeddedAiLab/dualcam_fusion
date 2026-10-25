#include "reconstruct/pyramid_reconstruct.h"

#include <opencv2/imgproc.hpp>

void reconstructFromLaplacian(const std::vector<cv::Mat> &lap, cv::Mat &out)
{
    const int levels = (int)lap.size();

    cv::Mat current = lap[levels - 1];        /* start with the base */
    cv::Mat up;

    for (int i = levels - 2; i >= 0; i--)
    {
        cv::pyrUp(current, up, lap[i].size()); /* enlarge to the next size  */
        current = up + lap[i];                 /* add the detail back       */
    }

    /* float -> 8-bit with saturation (values are clipped to 0..255) */
    current.convertTo(out, CV_8UC3);
}
