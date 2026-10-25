/*
 * image_preprocess.h  -  NV12 camera frame  ->  small BGR image.
 *
 * One RGA call does three jobs at once:
 *   1. crop     (centre crop to the processing aspect ratio, e.g. 16:9)
 *   2. resize   (4K / 2K  ->  PROC_WIDTH x PROC_HEIGHT)
 *   3. convert  (NV12     ->  BGR888)
 *
 * The result is written straight into a DMA-BUF, which the CPU can read
 * afterwards. If RGA is not available (or fails) a slower OpenCV CPU
 * version is used instead, so the program still works.
 */
#ifndef IMAGE_PREPROCESS_H
#define IMAGE_PREPROCESS_H

#include <opencv2/core.hpp>

#include "camera/camera.h"
#include "dmabuf/dmabuf.h"

class ImagePreprocess
{
public:
    ImagePreprocess(int dstWidth, int dstHeight);

    /* Convert `src` into `dst`. `dst` must hold dstWidth*dstHeight*3
     * bytes. Returns false only if both RGA and the CPU path failed.  */
    bool convert(const RawFrame &src, DmaBuffer &dst);

private:
    bool convertRga(const RawFrame &src, DmaBuffer &dst, const cv::Rect &crop);
    bool convertCpu(const RawFrame &src, DmaBuffer &dst, const cv::Rect &crop);

    int  dstW_;
    int  dstH_;
    bool rgaBroken_ = false;     /* after a failure we stop trying RGA */
};

#endif /* IMAGE_PREPROCESS_H */
