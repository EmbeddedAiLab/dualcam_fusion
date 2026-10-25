/*
 * camera.h  -  V4L2 capture of ONE camera using DMA-BUF buffers.
 *
 * Each Camera object owns:
 *   - the /dev/videoX capture node and the /dev/v4l-subdevX sensor node
 *   - its own pool of DMA-BUF buffers
 *   - its own capture thread
 *
 * The thread runs at the speed of the sensor and never waits for any
 * other camera. For every captured frame it calls your callback, and
 * as soon as the callback returns the buffer goes back to the driver.
 *
 * Because the buffer is returned right after the callback, the callback
 * must finish using the frame (e.g. let RGA convert it) before it
 * returns. This keeps the buffer ownership very simple.
 */
#ifndef CAMERA_H
#define CAMERA_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <linux/videodev2.h>

#include "dmabuf/dmabuf.h"

/* One captured frame, valid ONLY during the callback. */
struct RawFrame
{
    int            fd;          /* DMA-BUF fd (give this to RGA)          */
    const uint8_t *data;        /* CPU pointer to the same memory         */
    uint32_t       width;       /* visible width in pixels                */
    uint32_t       height;      /* visible height in pixels               */
    uint32_t       stride;      /* bytes per row (>= width)               */
    uint32_t       hstride;     /* rows in the Y plane (>= height)        */
    uint32_t       bytesUsed;   /* valid bytes in the buffer              */
    uint32_t       sequence;    /* driver frame counter                   */
    uint64_t       timestampUs; /* driver timestamp (CLOCK_MONOTONIC)     */
};

using FrameCallback = std::function<void(const RawFrame &)>;

class Camera
{
public:
    explicit Camera(const std::string &name);
    ~Camera();

    Camera(const Camera &) = delete;
    Camera &operator=(const Camera &) = delete;

    /* Step 1: open the nodes and negotiate the NV12 format. */
    bool open(const char *videoDevice,
              const char *sensorDevice,
              uint32_t width,
              uint32_t height,
              uint32_t pixelFormat);

    /* Step 2: allocate buffers, give them to V4L2, start streaming and
     * start the capture thread. `cb` is called from that thread.      */
    bool start(const char *heapPath, uint32_t bufferCount, FrameCallback cb);

    /* Stop the thread and the stream (also done by the destructor). */
    void stop();

    /* Format actually negotiated with the driver */
    uint32_t width() const   { return width_; }
    uint32_t height() const  { return height_; }
    uint32_t stride() const  { return stride_; }
    uint32_t hstride() const { return hstride_; }
    const std::string &name() const { return name_; }

    /* Optional sensor controls on the /dev/v4l-subdevX node
     * (exposure, gain, ...). Not used by default.            */
    bool setControl(uint32_t id, int32_t value);
    bool getControl(uint32_t id, int32_t *value);

private:
    bool negotiateFormat(uint32_t w, uint32_t h, uint32_t fourcc);
    bool queueBuffer(uint32_t index);
    void captureLoop();

    std::string name_;

    int videoFd_  = -1;
    int sensorFd_ = -1;

    bool               multiPlanar_ = false;  /* MPLANE or single-plane API */
    enum v4l2_buf_type type_        = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    uint32_t width_      = 0;
    uint32_t height_     = 0;
    uint32_t stride_     = 0;
    uint32_t hstride_    = 0;
    uint32_t bufferSize_ = 0;   /* V4L2 "sizeimage" */

    std::vector<std::unique_ptr<DmaBuffer>> buffers_;
    uint32_t usableBuffers_ = 0;

    FrameCallback     callback_;
    std::thread       thread_;
    std::atomic<bool> running_{false};
    bool              streaming_ = false;
};

#endif /* CAMERA_H */
