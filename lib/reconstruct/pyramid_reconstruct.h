/*
 * pyramid_reconstruct.h  -  turn a Laplacian pyramid back into an image.
 *
 * Start at the smallest level (base) and repeat:
 *     image = enlarge(image) + band[i]
 * until level 0 is reached.
 */
#ifndef PYRAMID_RECONSTRUCT_H
#define PYRAMID_RECONSTRUCT_H

#include <vector>
#include <opencv2/core.hpp>

/* `lap` : Laplacian pyramid (CV_32FC3)
 * `out` : reconstructed image converted to 8-bit BGR (CV_8UC3) */
void reconstructFromLaplacian(const std::vector<cv::Mat> &lap, cv::Mat &out);

#endif /* PYRAMID_RECONSTRUCT_H */
