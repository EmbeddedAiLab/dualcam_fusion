/*
 * frame_sync.h  -  frame "mailboxes" and timestamp pairing.
 *
 *  FrameStore : a small ring buffer of the most recent processed frames
 *               of ONE camera. The capture thread pushes into it, the
 *               main thread reads from it. Old frames are simply
 *               overwritten - the camera never waits for anybody.
 *
 *  FrameSync  : looks at two FrameStores and builds pairs:
 *               "newest frame of the SLOW camera" + "frame of the FAST
 *               camera that is closest in time to it".
 *
 *   IMX415:  A0 -- A1 -- A2 -- A3 -- A4 -- A5        (fast, ~60 fps)
 *   OV5647:  B0 ------------ B1 ------------ B2      (slow, ~15 fps)
 *                 pair(B1, closest A) ...
 */
#ifndef FRAME_SYNC_H
#define FRAME_SYNC_H

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "camera/camera.h"
#include "dmabuf/dmabuf.h"
#include "preprocess/image_preprocess.h"
#include "stats/pipeline_stats.h"

/* A frame that has already been converted by RGA (BGR, processing size).
 * `image` is a private copy, safe to use for as long as you like.       */
struct ProcessedFrame
{
    cv::Mat  image;                /* CV_8UC3, PROC_WIDTH x PROC_HEIGHT        */
    uint64_t timestampUs = 0;      /* time used for pairing (sensor time)      */
    uint64_t arrivalUs   = 0;      /* time the frame reached the application   */
    uint32_t sequence    = 0;      /* driver frame counter                     */
};

/* ----------------------------------------------------------------------
 * FrameStore
 * --------------------------------------------------------------------*/
class FrameStore
{
public:
    /* Allocate `slotCount` output buffers of width x height BGR. */
    bool init(const std::string &name, int slotCount, int width, int height);

    /* Called from the CAPTURE thread: convert the raw frame with RGA into
     * the oldest slot (overwriting it) and publish it.                  */
    void push(const RawFrame &raw, ImagePreprocess &pre);

    /* Wait until more frames than `lastCount` have been pushed (or the
     * timeout expires). Returns the current total number of pushes.     */
    uint64_t waitForNewer(uint64_t lastCount, int timeoutMs);

    /* Copy the newest frame. Returns false if the store is empty. */
    bool copyNewest(ProcessedFrame &out);

    /* Copy the frame whose timestamp is closest to `timestampUs`. */
    bool copyClosest(uint64_t timestampUs, ProcessedFrame &out);

private:
    struct Slot
    {
        std::unique_ptr<DmaBuffer> buf;     /* RGA writes here               */
        cv::Mat   view;                     /* cv::Mat wrapped around buf    */
        uint64_t  timestampUs = 0;
        uint64_t  arrivalUs   = 0;
        uint32_t  sequence    = 0;
        bool      valid       = false;
    };

    void copySlot(Slot &s, ProcessedFrame &out);

    std::string       name_;
    std::vector<Slot> slots_;
    size_t            next_   = 0;          /* slot that will be written next */
    size_t            newest_ = 0;          /* slot written most recently     */
    uint64_t          count_  = 0;          /* total frames pushed            */
    bool              clockWarned_ = false;

    std::mutex              mutex_;
    std::condition_variable cond_;
};

/* ----------------------------------------------------------------------
 * FrameSync
 * --------------------------------------------------------------------*/
struct FramePair
{
    ProcessedFrame a;          /* fast camera (IMX415)  */
    ProcessedFrame b;          /* slow camera (OV5647)  */
    int64_t        dtUs = 0;   /* a.timestamp - b.timestamp */
};

enum class PairResult
{
    Ok,         /* `out` holds a valid pair                     */
    Timeout,    /* no new slow-camera frame arrived in time     */
    Rejected    /* frames existed but did not fit (counted)     */
};

class FrameSync
{
public:
    FrameSync(FrameStore &fast, FrameStore &slow, PipelineStats &stats);

    /* Wait up to timeoutMs for a new slow-camera frame, then build a pair. */
    PairResult next(FramePair &out, int timeoutMs);

private:
    FrameStore    &fast_;
    FrameStore    &slow_;
    PipelineStats &stats_;
    uint64_t       lastSlowCount_ = 0;
};

#endif /* FRAME_SYNC_H */
