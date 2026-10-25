#include "calibration/camera_calibration.h"
#include "config/config.h"

#include <cstdio>
#include <unistd.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

CameraCalibration::CameraCalibration(int width, int height)
    : width_(width), height_(height)
{
    if (loadFromFile(cfg::CALIB_FILE))
    {
        fromFile_ = true;
        std::printf("[calibration] loaded homography from %s\n", cfg::CALIB_FILE);
    }
    else
    {
        buildDefaultTransform();
        std::printf("[calibration] %s not found - using default transform from config.h%s\n",
                    cfg::CALIB_FILE, identity_ ? " (identity: no warp)" : "");
    }

    buildInvalidMask();
}

/* Try to read the 3x3 homography from a YAML/XML file. */
bool CameraCalibration::loadFromFile(const char *path)
{
    /* Check first, so OpenCV does not print an error when the file is absent */
    if (::access(path, R_OK) != 0)
        return false;

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;

    cv::Mat H;
    fs["homography_b_to_a"] >> H;
    if (H.empty() || H.rows != 3 || H.cols != 3)
    {
        std::fprintf(stderr, "[calibration] %s has no valid 3x3 'homography_b_to_a'\n", path);
        return false;
    }

    H.convertTo(H_, CV_64F);
    identity_ = (cv::norm(H_, cv::Mat::eye(3, 3, CV_64F)) < 1e-9);
    return true;
}

/* Simple scale + rotation around the image centre, then a shift. */
void CameraCalibration::buildDefaultTransform()
{
    identity_ = (cfg::B_SCALE == 1.0 && cfg::B_ROTATE_DEG == 0.0 &&
                 cfg::B_SHIFT_X == 0.0 && cfg::B_SHIFT_Y == 0.0);

    cv::Point2f centre(width_ / 2.0f, height_ / 2.0f);
    cv::Mat M = cv::getRotationMatrix2D(centre, cfg::B_ROTATE_DEG, cfg::B_SCALE); /* 2x3 */
    M.at<double>(0, 2) += cfg::B_SHIFT_X;
    M.at<double>(1, 2) += cfg::B_SHIFT_Y;

    /* Make it a 3x3 matrix: add the row [0 0 1] */
    H_ = cv::Mat::eye(3, 3, CV_64F);
    M.copyTo(H_(cv::Rect(0, 0, 3, 2)));
}

/* Warp a white image once to learn which pixels will have valid data. */
void CameraCalibration::buildInvalidMask()
{
    if (identity_)
    {
        invalidMask_.release();              /* nothing is invalid */
        return;
    }

    cv::Mat white(height_, width_, CV_8UC1, cv::Scalar(255));
    cv::Mat valid;
    cv::warpPerspective(white, valid, H_, cv::Size(width_, height_),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

    /* Shrink the valid area by 2 px so blended edge pixels are not used */
    cv::erode(valid, valid, cv::Mat(), cv::Point(-1, -1), 2);

    cv::compare(valid, 0, invalidMask_, cv::CMP_EQ);   /* 255 where invalid */
}

void CameraCalibration::registerB(const cv::Mat &a, const cv::Mat &b, cv::Mat &bOut)
{
    if (identity_)
    {
        bOut = b;                            /* no warp needed */
        return;
    }

    /* Warp B into A's coordinate system */
    cv::warpPerspective(b, bOut, H_, cv::Size(width_, height_),
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));

    /* Fill the invalid border with A so the fusion keeps A there */
    a.copyTo(bOut, invalidMask_);
}
