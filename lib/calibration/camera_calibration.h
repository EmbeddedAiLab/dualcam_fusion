/*
 * camera_calibration.h  -  image registration (align B onto A).
 *
 * Both images already have the same size (RGA did that). Registration
 * warps camera B with a 3x3 homography so that it overlaps camera A.
 *
 * Where does the homography come from?
 *   1. "calibration.yml" if it exists (key: homography_b_to_a)
 *   2. otherwise a simple scale / rotate / shift from config.h
 *
 * Pixels of B that fall outside the warped image are "invalid". They are
 * filled with the pixels of A, so the fusion simply keeps A there. This
 * is the "common ROI / validity mask" step of the pipeline.
 */
#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <opencv2/core.hpp>

class CameraCalibration
{
public:
    CameraCalibration(int width, int height);

    /* Warp `b` onto `a`'s view. Result in `bOut` (same size as inputs). */
    void registerB(const cv::Mat &a, const cv::Mat &b, cv::Mat &bOut);

    bool usingFile() const { return fromFile_; }
    bool isIdentity() const { return identity_; }

private:
    bool loadFromFile(const char *path);
    void buildDefaultTransform();
    void buildInvalidMask();

    int      width_;
    int      height_;
    cv::Mat  H_;             /* 3x3, CV_64F, maps B coordinates -> A coordinates */
    bool     identity_  = true;
    bool     fromFile_  = false;
    cv::Mat  invalidMask_;   /* CV_8UC1, 255 where warped B has no data */
};

#endif /* CAMERA_CALIBRATION_H */
