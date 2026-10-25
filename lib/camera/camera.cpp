#include "camera/camera.h"
#include "config/config.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define CLEAR(x) std::memset(&(x), 0, sizeof(x))

/* ioctl that retries when interrupted by a signal */
static int xioctl(int fd, unsigned long request, void *arg)
{
    int r;
    do
    {
        r = ::ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

Camera::Camera(const std::string &name) : name_(name) {}

Camera::~Camera()
{
    stop();

    if (videoFd_ >= 0)  ::close(videoFd_);
    if (sensorFd_ >= 0) ::close(sensorFd_);
    /* buffers_ are freed automatically after the stream is off */
}

/* ----------------------------------------------------------------------
 * open(): open both device nodes and negotiate the format
 * --------------------------------------------------------------------*/
bool Camera::open(const char *videoDevice,
                  const char *sensorDevice,
                  uint32_t width,
                  uint32_t height,
                  uint32_t pixelFormat)
{
    /* Capture node. O_NONBLOCK so that we can never hang on shutdown. */
    videoFd_ = ::open(videoDevice, O_RDWR | O_NONBLOCK);
    if (videoFd_ < 0)
    {
        std::fprintf(stderr, "[%s] cannot open %s: %s\n",
                     name_.c_str(), videoDevice, std::strerror(errno));
        return false;
    }

    /* Sensor sub-device. Only needed for exposure/gain controls, so a
     * failure here is just a warning.                                  */
    sensorFd_ = ::open(sensorDevice, O_RDWR);
    if (sensorFd_ < 0)
    {
        std::fprintf(stderr, "[%s] warning: cannot open %s: %s\n",
                     name_.c_str(), sensorDevice, std::strerror(errno));
    }

    return negotiateFormat(width, height, pixelFormat);
}

/* ----------------------------------------------------------------------
 * negotiateFormat(): tell the driver what we want, read back what we got
 * --------------------------------------------------------------------*/
bool Camera::negotiateFormat(uint32_t w, uint32_t h, uint32_t fourcc)
{
    /* Does this node use the single-plane or multi-plane API? */
    struct v4l2_capability cap;
    CLEAR(cap);
    if (xioctl(videoFd_, VIDIOC_QUERYCAP, &cap) < 0)
    {
        std::perror("VIDIOC_QUERYCAP");
        return false;
    }

    multiPlanar_ = !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE);
    type_ = multiPlanar_ ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                         : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    /* Ask for width x height in NV12 */
    struct v4l2_format fmt;
    CLEAR(fmt);
    fmt.type = type_;

    if (multiPlanar_)
    {
        fmt.fmt.pix_mp.width       = w;
        fmt.fmt.pix_mp.height      = h;
        fmt.fmt.pix_mp.pixelformat = fourcc;
        fmt.fmt.pix_mp.field       = V4L2_FIELD_NONE;
    }
    else
    {
        fmt.fmt.pix.width       = w;
        fmt.fmt.pix.height      = h;
        fmt.fmt.pix.pixelformat = fourcc;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    }

    if (xioctl(videoFd_, VIDIOC_S_FMT, &fmt) < 0)
    {
        std::perror("VIDIOC_S_FMT");
        return false;
    }

    /* Read back what the driver really chose (it may pad the stride). */
    CLEAR(fmt);
    fmt.type = type_;
    if (xioctl(videoFd_, VIDIOC_G_FMT, &fmt) < 0)
    {
        std::perror("VIDIOC_G_FMT");
        return false;
    }

    uint32_t gotFormat;
    if (multiPlanar_)
    {
        gotFormat   = fmt.fmt.pix_mp.pixelformat;
        width_      = fmt.fmt.pix_mp.width;
        height_     = fmt.fmt.pix_mp.height;
        stride_     = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
        bufferSize_ = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

        if (fmt.fmt.pix_mp.num_planes != 1)
        {
            std::fprintf(stderr, "[%s] expected 1 plane (NV12), driver gave %u\n",
                         name_.c_str(), fmt.fmt.pix_mp.num_planes);
            return false;
        }
    }
    else
    {
        gotFormat   = fmt.fmt.pix.pixelformat;
        width_      = fmt.fmt.pix.width;
        height_     = fmt.fmt.pix.height;
        stride_     = fmt.fmt.pix.bytesperline;
        bufferSize_ = fmt.fmt.pix.sizeimage;
    }

    if (gotFormat != fourcc)
    {
        std::fprintf(stderr, "[%s] driver did not accept the requested pixel format\n",
                     name_.c_str());
        return false;
    }

    /* NV12 = Y plane (stride x hstride) followed by the UV plane (half
     * as many rows). So  sizeimage = stride * hstride * 3 / 2  and we can
     * recover hstride (the padded height) from sizeimage.               */
    if (stride_ == 0)
        stride_ = width_;
    hstride_ = (uint32_t)((uint64_t)bufferSize_ * 2 / (3ULL * stride_));
    if (hstride_ < height_)
        hstride_ = height_;

    if (width_ != w || height_ != h)
    {
        std::fprintf(stderr, "[%s] warning: asked %ux%u, driver gave %ux%u\n",
                     name_.c_str(), w, h, width_, height_);
    }

    std::printf("[%s] format %ux%u NV12, stride=%u, hstride=%u, sizeimage=%u\n",
                name_.c_str(), width_, height_, stride_, hstride_, bufferSize_);
    return true;
}

/* ----------------------------------------------------------------------
 * start(): buffers -> V4L2 -> STREAMON -> capture thread
 * --------------------------------------------------------------------*/
bool Camera::start(const char *heapPath, uint32_t bufferCount, FrameCallback cb)
{
    if (videoFd_ < 0 || !cb || bufferCount < 2)
        return false;

    callback_ = cb;

    /* 1. Allocate the DMA-BUF pool. Each buffer is the driver's
     *    "sizeimage" plus a safety margin (see config.h). The full
     *    allocated size is what we later report as the V4L2 length.   */
    const size_t allocSize = (size_t)bufferSize_ + cfg::CAMERA_BUFFER_MARGIN;

    buffers_.clear();
    for (uint32_t i = 0; i < bufferCount; i++)
    {
        auto b = std::make_unique<DmaBuffer>();
        if (!b->allocate(heapPath, allocSize))
        {
            std::fprintf(stderr, "[%s] cannot allocate buffer %u\n", name_.c_str(), i);
            return false;
        }
        buffers_.push_back(std::move(b));
    }

    /* 2. Tell V4L2 that we will provide the memory (DMABUF import mode) */
    struct v4l2_requestbuffers req;
    CLEAR(req);
    req.count  = bufferCount;
    req.type   = type_;
    req.memory = V4L2_MEMORY_DMABUF;

    if (xioctl(videoFd_, VIDIOC_REQBUFS, &req) < 0)
    {
        std::perror("VIDIOC_REQBUFS");
        return false;
    }

    usableBuffers_ = (req.count < bufferCount) ? req.count : bufferCount;
    if (usableBuffers_ < 2)
    {
        std::fprintf(stderr, "[%s] driver granted too few buffers\n", name_.c_str());
        return false;
    }

    /* 3. Queue every buffer so the driver can start filling them */
    for (uint32_t i = 0; i < usableBuffers_; i++)
    {
        if (!queueBuffer(i))
            return false;
    }

    /* 4. Start streaming */
    enum v4l2_buf_type t = type_;
    if (xioctl(videoFd_, VIDIOC_STREAMON, &t) < 0)
    {
        std::perror("VIDIOC_STREAMON");
        return false;
    }
    streaming_ = true;

    /* 5. Start the capture thread */
    running_ = true;
    thread_  = std::thread(&Camera::captureLoop, this);

    std::printf("[%s] streaming with %u buffers\n", name_.c_str(), usableBuffers_);
    return true;
}

/* ----------------------------------------------------------------------
 * queueBuffer(): hand one buffer to the driver (QBUF)
 *
 * NOTE (learned from your working code): this driver needs
 * `bytesused` and `length` filled even for capture buffers, otherwise
 * the re-queue is accepted but the buffer is never re-armed.
 * --------------------------------------------------------------------*/
bool Camera::queueBuffer(uint32_t index)
{
    struct v4l2_buffer buf;
    struct v4l2_plane  planes[1];
    CLEAR(buf);
    CLEAR(planes);

    const uint32_t size = (uint32_t)buffers_[index]->size();
    const int      fd   = buffers_[index]->fd();

    buf.type   = type_;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index  = index;
    buf.field  = V4L2_FIELD_NONE;

    if (multiPlanar_)
    {
        buf.length   = 1;               /* number of planes */
        buf.m.planes = planes;
        planes[0].m.fd        = fd;
        planes[0].length      = size;
        planes[0].bytesused   = size;
        planes[0].data_offset = 0;
    }
    else
    {
        buf.m.fd      = fd;
        buf.length    = size;
        buf.bytesused = size;
    }

    if (xioctl(videoFd_, VIDIOC_QBUF, &buf) < 0)
    {
        std::fprintf(stderr, "[%s] QBUF %u failed: %s (fd=%d, length=%u, sizeimage=%u)\n",
                     name_.c_str(), index, std::strerror(errno), fd, size, bufferSize_);
        return false;
    }
    return true;
}

/* ----------------------------------------------------------------------
 * captureLoop(): runs in its own thread, at the sensor's own speed
 * --------------------------------------------------------------------*/
void Camera::captureLoop()
{
    while (running_)
    {
        /* Wait (max 200 ms) until a frame is ready. The timeout lets us
         * notice running_ == false quickly.                              */
        struct pollfd pfd;
        pfd.fd      = videoFd_;
        pfd.events  = POLLIN | POLLRDNORM;
        pfd.revents = 0;

        int pr = ::poll(&pfd, 1, 200);
        if (pr == 0)
            continue;                         /* timeout */
        if (pr < 0)
        {
            if (errno == EINTR)
                continue;
            std::perror("poll");
            break;
        }

        /* Take the finished buffer from the driver (DQBUF) */
        struct v4l2_buffer buf;
        struct v4l2_plane  planes[1];
        CLEAR(buf);
        CLEAR(planes);
        buf.type   = type_;
        buf.memory = V4L2_MEMORY_DMABUF;
        if (multiPlanar_)
        {
            buf.m.planes = planes;
            buf.length   = 1;
        }

        if (xioctl(videoFd_, VIDIOC_DQBUF, &buf) < 0)
        {
            if (errno == EAGAIN)
                continue;
            std::perror("VIDIOC_DQBUF");
            break;
        }

        if (buf.index >= usableBuffers_)
        {
            std::fprintf(stderr, "[%s] driver returned bad index %u\n",
                         name_.c_str(), buf.index);
            break;
        }

        /* Hand the frame to the user, unless the driver flagged an error */
        if (!(buf.flags & V4L2_BUF_FLAG_ERROR))
        {
            DmaBuffer &db = *buffers_[buf.index];

            RawFrame f;
            f.fd          = db.fd();
            f.data        = db.data();
            f.width       = width_;
            f.height      = height_;
            f.stride      = stride_;
            f.hstride     = hstride_;
            f.bytesUsed   = multiPlanar_ ? planes[0].bytesused : buf.bytesused;
            f.sequence    = buf.sequence;
            f.timestampUs = (uint64_t)buf.timestamp.tv_sec * 1000000ULL +
                            (uint64_t)buf.timestamp.tv_usec;

            callback_(f);   /* user finishes with the buffer in here */
        }

        /* Give the buffer back to the driver immediately */
        if (!queueBuffer(buf.index))
            break;
    }

    running_ = false;
}

/* ----------------------------------------------------------------------
 * stop()
 * --------------------------------------------------------------------*/
void Camera::stop()
{
    running_ = false;

    if (thread_.joinable())
        thread_.join();

    if (streaming_)
    {
        enum v4l2_buf_type t = type_;
        xioctl(videoFd_, VIDIOC_STREAMOFF, &t);
        streaming_ = false;
    }
}

/* ----------------------------------------------------------------------
 * Optional sensor controls (on the sub-device node)
 * --------------------------------------------------------------------*/
bool Camera::setControl(uint32_t id, int32_t value)
{
    if (sensorFd_ < 0)
        return false;

    struct v4l2_control c;
    CLEAR(c);
    c.id    = id;
    c.value = value;
    return xioctl(sensorFd_, VIDIOC_S_CTRL, &c) == 0;
}

bool Camera::getControl(uint32_t id, int32_t *value)
{
    if (sensorFd_ < 0 || !value)
        return false;

    struct v4l2_control c;
    CLEAR(c);
    c.id = id;
    if (xioctl(sensorFd_, VIDIOC_G_CTRL, &c) < 0)
        return false;

    *value = c.value;
    return true;
}