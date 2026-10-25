#include "preprocess/image_preprocess.h"
#include "config/config.h"

#include <cstdio>
#include <cstring>
#include <opencv2/imgproc.hpp>

#ifdef HAVE_RGA
#include "im2d.hpp"        /* Rockchip librga (im2d API) */
#endif

ImagePreprocess::ImagePreprocess(int dstWidth, int dstHeight)
    : dstW_(dstWidth), dstH_(dstHeight)
{
#ifdef HAVE_RGA
    std::printf("[preprocess] using Rockchip RGA\n");
#else
    std::printf("[preprocess] RGA not compiled in - using OpenCV CPU path (slow)\n");
#endif
}

/* ----------------------------------------------------------------------
 * Largest centred rectangle of the source that has the same aspect ratio
 * as the destination. Values are kept even (NV12 requires it).
 * Example: OV5647 2592x1944 (4:3) -> 2592x1458 (16:9).
 * IMX415 3840x2160 is already 16:9, so nothing is cropped.
 * --------------------------------------------------------------------*/
static cv::Rect centreCrop(int srcW, int srcH, int dstW, int dstH)
{
    int cw = srcW;
    int ch = srcW * dstH / dstW;
    if (ch > srcH)
    {
        ch = srcH;
        cw = srcH * dstW / dstH;
    }
    cw &= ~1;
    ch &= ~1;
    int x = ((srcW - cw) / 2) & ~1;
    int y = ((srcH - ch) / 2) & ~1;
    return cv::Rect(x, y, cw, ch);
}

bool ImagePreprocess::convert(const RawFrame &src, DmaBuffer &dst)
{
    if (dst.size() < (size_t)dstW_ * dstH_ * 3)
        return false;

    cv::Rect crop = centreCrop((int)src.width, (int)src.height, dstW_, dstH_);

    if (!rgaBroken_ && convertRga(src, dst, crop))
        return true;

    return convertCpu(src, dst, crop);
}

/* ----------------------------------------------------------------------
 * RGA path (hardware, no CPU load)
 * --------------------------------------------------------------------*/
bool ImagePreprocess::convertRga(const RawFrame &src, DmaBuffer &dst, const cv::Rect &crop)
{
#ifdef HAVE_RGA
    /* Describe the source: the camera DMA-BUF, NV12, with its real
     * stride (wstride) and padded height (hstride).                   */
    rga_buffer_t srcBuf = wrapbuffer_fd_t(src.fd,
                                          (int)src.width, (int)src.height,
                                          (int)src.stride, (int)src.hstride,
                                          RK_FORMAT_YCbCr_420_SP);

    /* Describe the destination: our DMA-BUF, BGR888, small size. */
    int dstFormat = cfg::RGA_SWAP_RB ? RK_FORMAT_RGB_888 : RK_FORMAT_BGR_888;
    rga_buffer_t dstBuf = wrapbuffer_fd(dst.fd(), dstW_, dstH_, dstFormat);

    rga_buffer_t pat;                       /* unused pattern buffer */
    std::memset(&pat, 0, sizeof(pat));

    im_rect srcRect = { crop.x, crop.y, crop.width, crop.height };
    im_rect dstRect = { 0, 0, dstW_, dstH_ };
    im_rect patRect;
    std::memset(&patRect, 0, sizeof(patRect));

    /* One call: crop + scale + colour convert. IM_SYNC = wait until done. */
    IM_STATUS st = improcess(srcBuf, dstBuf, pat, srcRect, dstRect, patRect, IM_SYNC);
    if (st != IM_STATUS_SUCCESS)
    {
        std::fprintf(stderr, "[preprocess] RGA failed (%s) - switching to CPU path\n",
                     imStrError(st));
        rgaBroken_ = true;
        return false;
    }
    return true;
#else
    (void)src; (void)dst; (void)crop;
    return false;
#endif
}

/* ----------------------------------------------------------------------
 * CPU fallback (OpenCV). Slow for 4K, but good for testing without RGA.
 * Reads the camera buffer through its CPU mapping.
 * --------------------------------------------------------------------*/
bool ImagePreprocess::convertCpu(const RawFrame &src, DmaBuffer &dst, const cv::Rect &crop)
{
    /* View the whole NV12 buffer as one grey image:
     * `stride` columns, hstride*3/2 rows (Y rows + UV rows).          */
    cv::Mat yuv((int)(src.hstride * 3 / 2), (int)src.stride, CV_8UC1,
                const_cast<uint8_t *>(src.data));

    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);   /* stride x hstride, BGR */

    /* Crop the visible area, then resize into the destination buffer */
    cv::Mat cropped = bgr(crop);

    dst.cpuAccessBegin(true);
    cv::Mat dstView(dstH_, dstW_, CV_8UC3, dst.data());
    cv::resize(cropped, dstView, dstView.size(), 0, 0, cv::INTER_AREA);
    dst.cpuAccessEnd(true);
    return true;
}
