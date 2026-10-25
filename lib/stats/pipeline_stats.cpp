#include "stats/pipeline_stats.h"
#include "config/config.h"

#include <cstdio>

/* Read the "cpu" line of /proc/stat: returns busy and total jiffies. */
static bool readCpuTimes(uint64_t &busy, uint64_t &total)
{
    FILE *f = std::fopen("/proc/stat", "r");
    if (!f)
        return false;

    unsigned long long user = 0, nice = 0, sys = 0, idle = 0,
                       iowait = 0, irq = 0, softirq = 0, steal = 0;

    int n = std::fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                        &user, &nice, &sys, &idle, &iowait, &irq, &softirq, &steal);
    std::fclose(f);
    if (n < 4)
        return false;

    uint64_t idleAll = idle + iowait;
    total = user + nice + sys + idle + iowait + irq + softirq + steal;
    busy  = total - idleAll;
    return true;
}

/* Smooth a value a little so the overlay numbers do not flicker. */
static double smooth(double oldValue, double newValue)
{
    return (oldValue == 0.0) ? newValue : (0.9 * oldValue + 0.1 * newValue);
}

PipelineStats::PipelineStats()
{
    captured_[0] = captured_[1] = 0;
    fused_ = 0;
    for (auto &d : drops_)
        d = 0;

    lastCaptured_[0] = lastCaptured_[1] = 0;
    lastFused_ = 0;
    lastUs_    = nowUs();
    lastCpuBusy_ = lastCpuTotal_ = 0;
    readCpuTimes(lastCpuBusy_, lastCpuTotal_);
}

void PipelineStats::setLatencyMs(double ms) { snap_.latencyMs = smooth(snap_.latencyMs, ms); }
void PipelineStats::setPairDtMs(double ms)  { snap_.pairDtMs  = smooth(snap_.pairDtMs, ms); }
void PipelineStats::setStageMs(Stage s, double ms)
{
    snap_.stageMs[s] = smooth(snap_.stageMs[s], ms);
}

void PipelineStats::update()
{
    uint64_t now = nowUs();
    if (now - lastUs_ < 1000000ULL)
        return;                                   /* wait for a full second */

    double sec = (double)(now - lastUs_) / 1e6;

    uint64_t a = captured_[CAM_A], b = captured_[CAM_B], f = fused_;
    snap_.fpsA     = (double)(a - lastCaptured_[CAM_A]) / sec;
    snap_.fpsB     = (double)(b - lastCaptured_[CAM_B]) / sec;
    snap_.fpsFused = (double)(f - lastFused_) / sec;
    lastCaptured_[CAM_A] = a;
    lastCaptured_[CAM_B] = b;
    lastFused_ = f;

    uint64_t busy, total;
    if (readCpuTimes(busy, total) && total > lastCpuTotal_)
    {
        snap_.cpuPercent = 100.0 * (double)(busy - lastCpuBusy_) /
                           (double)(total - lastCpuTotal_);
        lastCpuBusy_  = busy;
        lastCpuTotal_ = total;
    }

    lastUs_ = now;
}

StatsSnapshot PipelineStats::snapshot() const
{
    StatsSnapshot s = snap_;
    for (int i = 0; i < DROP_REASON_COUNT; i++)
        s.drops[i] = drops_[i];
    s.fusedFrames = fused_;
    return s;
}
