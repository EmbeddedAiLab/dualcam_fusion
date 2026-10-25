/*
 * config.h  -  every tunable number of the project lives here.
 *
 * There are NO command-line arguments in this application.
 * If you want to change a node, a resolution or a threshold,
 * edit this file and rebuild.
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>
#include <ctime>
#include <linux/videodev2.h>

namespace cfg
{
/* ------------------------------------------------------------------
 * Camera device nodes (from your board)
 * ----------------------------------------------------------------*/
constexpr const char *IMX415_VIDEO  = "/dev/video31";
constexpr const char *IMX415_SENSOR = "/dev/v4l-subdev8";
constexpr const char *OV5647_VIDEO  = "/dev/video22";
constexpr const char *OV5647_SENSOR = "/dev/v4l-subdev3";

/* Capture resolutions. The driver may adjust them; the camera code
 * prints what it really got.                                       */
constexpr uint32_t IMX415_WIDTH  = 3840;
constexpr uint32_t IMX415_HEIGHT = 2160;
constexpr uint32_t OV5647_WIDTH  = 2592;
constexpr uint32_t OV5647_HEIGHT = 1944;

/* Both cameras deliver NV12 */
constexpr uint32_t PIXEL_FORMAT = V4L2_PIX_FMT_NV12;

/* ------------------------------------------------------------------
 * Memory (DMA heaps)
 * ----------------------------------------------------------------*/
/* Camera capture buffers: same heap that works in your other project */
constexpr const char *HEAP_CAMERA = "/dev/dma_heap/reserved";

/* RGA output buffers that the CPU reads afterwards. A cached heap is
 * much faster for CPU reads. If it does not exist on your image, the
 * code automatically falls back to HEAP_CAMERA.                      */
constexpr const char *HEAP_PROCESSING = "/dev/dma_heap/system";

/* Extra bytes added to every camera DMA-BUF on top of the driver's
 * "sizeimage". Some Rockchip drivers want a slightly larger buffer than
 * they report (stride/alignment padding). Same idea as
 * CAPD_BUFFER_SIZE_MARGIN in your other project.                       */
constexpr uint32_t CAMERA_BUFFER_MARGIN = 64 * 1024;

/* V4L2 buffers per camera (minimum 2, 4 is comfortable) */
constexpr uint32_t CAMERA_BUFFER_COUNT = 4;

/* ------------------------------------------------------------------
 * Common processing resolution (everything after RGA uses this)
 * ----------------------------------------------------------------*/
constexpr int PROC_WIDTH  = 960;
constexpr int PROC_HEIGHT = 540;

/* If colours look swapped (red <-> blue) on your librga version,
 * set this to true.                                                */
constexpr bool RGA_SWAP_RB = false;

/* ------------------------------------------------------------------
 * Frame history ("mailbox") per camera
 * IMX415 is fast, so it keeps more history to find the best match.
 * ----------------------------------------------------------------*/
constexpr int RING_SLOTS_A = 8;   /* IMX415, ~130 ms of history @60fps */
constexpr int RING_SLOTS_B = 3;   /* OV5647                            */

/* ------------------------------------------------------------------
 * Frame pairing limits
 * ----------------------------------------------------------------*/
/* Max time difference between the two frames of a pair */
constexpr int64_t MAX_PAIR_DT_US   = 40000;    /* 40 ms  */
/* Reject pairs whose slow-camera frame is older than this */
constexpr int64_t MAX_FRAME_AGE_US = 250000;   /* 250 ms */

/* ------------------------------------------------------------------
 * Calibration / registration (camera B is warped onto camera A)
 *
 * If "calibration.yml" exists in the working directory it is used
 * (key: homography_b_to_a, 3x3). Otherwise the simple default
 * transform below is used. Scale 1.0 and zeros = no warp at all.
 * ----------------------------------------------------------------*/
constexpr const char *CALIB_FILE = "calibration.yml";
constexpr double B_SCALE      = 1.0;   /* zoom of B around its centre   */
constexpr double B_ROTATE_DEG = 0.0;   /* rotation of B in degrees      */
constexpr double B_SHIFT_X    = 0.0;   /* shift of B in processing px   */
constexpr double B_SHIFT_Y    = 0.0;

/* ------------------------------------------------------------------
 * Pyramid
 * ----------------------------------------------------------------*/
constexpr int PYRAMID_LEVELS = 5;      /* 960x540 -> ... -> 60x34       */
}

/* ----------------------------------------------------------------------
 * Tiny helper: current time in microseconds on CLOCK_MONOTONIC.
 * V4L2 buffer timestamps use the same clock, so they can be compared.
 * --------------------------------------------------------------------*/
inline uint64_t nowUs()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

#endif /* CONFIG_H */