/*
 * pipeline_stats.h  -  FPS, latency, drops and CPU usage.
 *
 * Capture threads only touch the atomic counters. Everything else is
 * used from the main thread, so no locks are needed.
 */
#ifndef PIPELINE_STATS_H
#define PIPELINE_STATS_H

#include <atomic>
#include <cstdint>

enum CameraId   { CAM_A = 0, CAM_B = 1 };            /* A = IMX415, B = OV5647 */

enum DropReason
{
    DROP_SKIPPED_B = 0,   /* slow-camera frame skipped: fusion was too slow  */
    DROP_NO_MATCH  = 1,   /* no fast-camera frame close enough in time       */
    DROP_TOO_OLD   = 2,   /* frame was already too old when we got to it     */
    DROP_REASON_COUNT
};

enum Stage
{
    STAGE_REGISTER = 0,
    STAGE_PYRAMID,
    STAGE_FUSE,
    STAGE_RECONSTRUCT,
    STAGE_COUNT
};

/* Everything the viewer needs to print */
struct StatsSnapshot
{
    double   fpsA = 0, fpsB = 0, fpsFused = 0;
    double   latencyMs = 0;                 /* sensor time -> fused image ready */
    double   pairDtMs  = 0;                 /* time gap inside the last pair    */
    double   cpuPercent = 0;
    double   stageMs[STAGE_COUNT] = {0, 0, 0, 0};
    uint64_t drops[DROP_REASON_COUNT] = {0, 0, 0};
    uint64_t fusedFrames = 0;
};

class PipelineStats
{
public:
    PipelineStats();

    /* --- thread-safe counters (called from any thread) --- */
    void countCapture(CameraId cam) { captured_[cam]++; }
    void countFused()               { fused_++; }
    void countDrop(DropReason r, uint64_t n = 1) { drops_[r] += n; }

    /* --- main thread only --- */
    void setLatencyMs(double ms);
    void setPairDtMs(double ms);
    void setStageMs(Stage s, double ms);

    /* Call often (every loop). Recomputes FPS and CPU once per second. */
    void update();

    StatsSnapshot snapshot() const;

private:
    std::atomic<uint64_t> captured_[2];
    std::atomic<uint64_t> fused_;
    std::atomic<uint64_t> drops_[DROP_REASON_COUNT];

    /* values from the previous 1-second window */
    uint64_t lastCaptured_[2];
    uint64_t lastFused_;
    uint64_t lastUs_;
    uint64_t lastCpuBusy_, lastCpuTotal_;

    StatsSnapshot snap_;
};

#endif /* PIPELINE_STATS_H */
