/*
 * fusion_engine.h  -  fuse two Laplacian pyramids band by band.
 *
 * Rules:
 *   detail bands (all levels except the last):
 *       for every pixel keep the coefficient with the larger magnitude
 *       (= the camera that sees more detail / contrast there).
 *   base band (last level, low frequencies):
 *       average of the two cameras.
 */
#ifndef FUSION_ENGINE_H
#define FUSION_ENGINE_H

#include <vector>
#include <opencv2/core.hpp>

class FusionEngine
{
public:
    void fuse(const std::vector<cv::Mat> &lapA,
              const std::vector<cv::Mat> &lapB,
              std::vector<cv::Mat> &lapFused);

private:
    /* Scratch images reused every frame (avoids re-allocation) */
    cv::Mat magA_, magB_, absTmp_, mask_;
};

#endif /* FUSION_ENGINE_H */
