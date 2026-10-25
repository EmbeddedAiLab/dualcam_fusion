#include "sync/frame_sync.h"
#include "config/config.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>

/* ======================================================================
 * FrameStore
 * ====================================================================*/

bool FrameStore::init(const std::string &name, int slotCount, int width, int height)
{
    name_ = name;
    slots_.clear();
    slots_.resize(slotCount);

    const size_t bytes = (size_t)width * height * 3;

    for (auto &s : slots_)
    {
        s.buf = std::make_unique<DmaBuffer>();

        /* Prefer the cached heap (fast CPU reads); fall back to the
         * camera heap if it is not available.                         */
        if (!s.buf->allocate(cfg::HEAP_PROCESSING, bytes) &&
            !s.buf->allocate(cfg::HEAP_CAMERA, bytes))
        {
            std::fprintf(stderr, "[%s] cannot allocate frame store\n", name_.c_str());
            return false;
        }

        /* Wrap the DMA-BUF memory in a cv::Mat (no copy) */
        s.view = cv::Mat(height, width, CV_8UC3, s.buf->data());
    }
    return true;
}

void FrameStore::push(const RawFrame &raw, ImagePreprocess &pre)
{
    /* The lock is held during the RGA call so that a reader can never
     * look at a slot while it is being overwritten. It is short (a few
     * ms) and the reader only holds it for a quick memcpy.             */
    std::lock_guard<std::mutex> lock(mutex_);

    Slot &s = slots_[next_];
    s.valid = false;

    if (!pre.convert(raw, *s.buf))
        return;                                  /* conversion failed: drop */

    uint64_t now = nowUs();
    uint64_t ts  = raw.timestampUs;

    /* Safety net: pairing needs both cameras on the SAME clock. V4L2
     * timestamps should be CLOCK_MONOTONIC. If one is far from "now",
     * the driver uses another clock, so use the arrival time instead.  */
    uint64_t diff = (ts > now) ? (ts - now) : (now - ts);
    if (diff > 1000000ULL)
    {
        if (!clockWarned_)
        {
            std::fprintf(stderr,
                "[%s] warning: V4L2 timestamps are not CLOCK_MONOTONIC, "
                "using arrival time for pairing\n", name_.c_str());
            clockWarned_ = true;
        }
        ts = now;
    }

    s.timestampUs = ts;
    s.arrivalUs   = now;
    s.sequence    = raw.sequence;
    s.valid       = true;

    newest_ = next_;
    next_   = (next_ + 1) % slots_.size();
    count_++;

    cond_.notify_all();
}

uint64_t FrameStore::waitForNewer(uint64_t lastCount, int timeoutMs)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                   [&] { return count_ > lastCount; });
    return count_;
}

void FrameStore::copySlot(Slot &s, ProcessedFrame &out)
{
    /* RGA wrote this memory, so tell the cache before the CPU reads it */
    s.buf->cpuAccessBegin(false);
    s.view.copyTo(out.image);
    s.buf->cpuAccessEnd(false);

    out.timestampUs = s.timestampUs;
    out.arrivalUs   = s.arrivalUs;
    out.sequence    = s.sequence;
}

bool FrameStore::copyNewest(ProcessedFrame &out)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ == 0 || !slots_[newest_].valid)
        return false;

    copySlot(slots_[newest_], out);
    return true;
}

bool FrameStore::copyClosest(uint64_t timestampUs, ProcessedFrame &out)
{
    std::lock_guard<std::mutex> lock(mutex_);

    Slot    *best     = nullptr;
    uint64_t bestDiff = UINT64_MAX;

    for (auto &s : slots_)
    {
        if (!s.valid)
            continue;

        uint64_t d = (s.timestampUs > timestampUs) ? (s.timestampUs - timestampUs)
                                                   : (timestampUs - s.timestampUs);
        if (d < bestDiff)
        {
            bestDiff = d;
            best     = &s;
        }
    }

    if (!best)
        return false;

    copySlot(*best, out);
    return true;
}

/* ======================================================================
 * FrameSync
 * ====================================================================*/

FrameSync::FrameSync(FrameStore &fast, FrameStore &slow, PipelineStats &stats)
    : fast_(fast), slow_(slow), stats_(stats)
{
}

PairResult FrameSync::next(FramePair &out, int timeoutMs)
{
    /* 1. Wait for a NEW frame of the slow camera. The fast camera is
     *    never waited for - its frames are already in its ring.         */
    uint64_t count = slow_.waitForNewer(lastSlowCount_, timeoutMs);
    if (count <= lastSlowCount_)
        return PairResult::Timeout;

    /* If more than one slow frame arrived since last time, we were too
     * slow: those frames are skipped (we always use the newest).        */
    if (count - lastSlowCount_ > 1)
        stats_.countDrop(DROP_SKIPPED_B, count - lastSlowCount_ - 1);
    lastSlowCount_ = count;

    /* 2. Newest slow frame */
    if (!slow_.copyNewest(out.b))
        return PairResult::Rejected;

    /* 3. Fast-camera frame closest in time to it */
    if (!fast_.copyClosest(out.b.timestampUs, out.a))
    {
        stats_.countDrop(DROP_NO_MATCH);
        return PairResult::Rejected;
    }

    out.dtUs = (int64_t)out.a.timestampUs - (int64_t)out.b.timestampUs;

    /* 4. Reject bad pairs */
    if ((int64_t)(nowUs() - out.b.arrivalUs) > cfg::MAX_FRAME_AGE_US)
    {
        stats_.countDrop(DROP_TOO_OLD);
        return PairResult::Rejected;
    }

    if (std::llabs(out.dtUs) > cfg::MAX_PAIR_DT_US)
    {
        stats_.countDrop(DROP_NO_MATCH);
        return PairResult::Rejected;
    }

    return PairResult::Ok;
}
