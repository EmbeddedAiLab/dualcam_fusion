#include "pyramid/laplacian_pyramid.h"

#include <opencv2/imgproc.hpp>

void buildGaussianPyramid(const cv::Mat &image, int levels, std::vector<cv::Mat> &gauss)
{
    gauss.resize(levels);
    gauss[0] = image;                         /* level 0 = original (no copy) */

    for (int i = 1; i < levels; i++)
        cv::pyrDown(gauss[i - 1], gauss[i]);  /* blur + halve the size */
}

void buildLaplacianPyramid(const std::vector<cv::Mat> &gauss, std::vector<cv::Mat> &lap)
{
    const int levels = (int)gauss.size();
    lap.resize(levels);

    cv::Mat up;
    for (int i = 0; i < levels - 1; i++)
    {
        /* Enlarge the next (smaller) level back to this level's size.
         * Passing the exact size handles odd image dimensions.        */
        cv::pyrUp(gauss[i + 1], up, gauss[i].size());

        /* band = what the smaller level cannot describe = fine detail */
        cv::subtract(gauss[i], up, lap[i]);
    }

    lap[levels - 1] = gauss[levels - 1];      /* base level */
}
