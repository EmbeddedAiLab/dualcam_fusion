/*
 * laplacian_pyramid.h  -  Gaussian and Laplacian pyramids.
 *
 * Gaussian pyramid : level 0 = the image, every next level is blurred
 *                    and half the size.
 * Laplacian pyramid: band i = G[i] - enlarge(G[i+1])  (the detail that
 *                    is lost when going from level i to level i+1).
 *                    The LAST entry is simply the smallest Gaussian
 *                    image (the "base" / low frequencies).
 *
 * All images are CV_32FC3.
 */
#ifndef LAPLACIAN_PYRAMID_H
#define LAPLACIAN_PYRAMID_H

#include <vector>
#include <opencv2/core.hpp>

void buildGaussianPyramid(const cv::Mat &image,
                          int levels,
                          std::vector<cv::Mat> &gauss);

void buildLaplacianPyramid(const std::vector<cv::Mat> &gauss,
                           std::vector<cv::Mat> &lap);

#endif /* LAPLACIAN_PYRAMID_H */
