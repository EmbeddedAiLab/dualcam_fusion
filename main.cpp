/*
 * DualCam Fusion Lab  -  Radxa Rock 4D (RK3576)
 *
 *   IMX415 (fast, 4K)  ----+
 *                          +--> pair by timestamp --> register --> pyramids
 *   OV5647 (slow, 2K)  ----+                      --> fuse --> reconstruct
 *                                                 --> OpenCV window
 *
 * No command-line arguments. All settings are in lib/config/config.h.
 *
 * Threads
 *   - IMX415 capture thread : captures + RGA, runs at the sensor speed
 *   - OV5647 capture thread : captures + RGA, runs at the sensor speed
 *   - main thread           : pairing, registration, fusion, display
 * The capture threads never wait for the main thread or for each other.
 */

#include <csignal>
#include <cstdio>

#include <opencv2/core.hpp>

#include "config/config.h"
#include "camera/camera.h"
#include "preprocess/image_preprocess.h"
#include "sync/frame_sync.h"
#include "calibration/camera_calibration.h"
#include "pyramid/laplacian_pyramid.h"
#include "fusion/fusion_engine.h"
#include "reconstruct/pyramid_reconstruct.h"
#include "stats/pipeline_stats.h"
#include "viewer/viewer.h"

/* Set to 0 by Ctrl+C so the main loop can exit cleanly */
static volatile sig_atomic_t g_running = 1;
static void onSignal(int) { g_running = 0; }

/* Milliseconds between two nowUs() values */
static double elapsedMs(uint64_t t0, uint64_t t1) { return (double)(t1 - t0) / 1000.0; }

int main()
{
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::printf("=== DualCam Fusion Lab ===\n");

    const int W = cfg::PROC_WIDTH;
    const int H = cfg::PROC_HEIGHT;

    PipelineStats stats;

    /* ------------------------------------------------------------------
     * 1. Frame stores ("mailboxes") and RGA converters, one per camera
     * ----------------------------------------------------------------*/
    FrameStore storeA;   /* IMX415 */
    FrameStore storeB;   /* OV5647 */

    if (!storeA.init("IMX415 store", cfg::RING_SLOTS_A, W, H) ||
        !storeB.init("OV5647 store", cfg::RING_SLOTS_B, W, H))
        return 1;

    ImagePreprocess preA(W, H);
    ImagePreprocess preB(W, H);

    /* ------------------------------------------------------------------
     * 2. Open both cameras (negotiates NV12 with each driver)
     * ----------------------------------------------------------------*/
    Camera camA("IMX415");
    Camera camB("OV5647");

    if (!camA.open(cfg::IMX415_VIDEO, cfg::IMX415_SENSOR,
                   cfg::IMX415_WIDTH, cfg::IMX415_HEIGHT, cfg::PIXEL_FORMAT))
        return 1;

    if (!camB.open(cfg::OV5647_VIDEO, cfg::OV5647_SENSOR,
                   cfg::OV5647_WIDTH, cfg::OV5647_HEIGHT, cfg::PIXEL_FORMAT))
        return 1;

    /* ------------------------------------------------------------------
     * 3. Start streaming. The callback runs in each camera's own thread:
     *    count the frame, let RGA convert it into the mailbox, return.
     *    Right after the callback the V4L2 buffer goes back to the driver.
     * ----------------------------------------------------------------*/
    bool ok = camA.start(cfg::HEAP_CAMERA, cfg::CAMERA_BUFFER_COUNT,
                         [&](const RawFrame &f)
                         {
                             stats.countCapture(CAM_A);
                             storeA.push(f, preA);
                         });

    ok = ok && camB.start(cfg::HEAP_CAMERA, cfg::CAMERA_BUFFER_COUNT,
                          [&](const RawFrame &f)
                          {
                              stats.countCapture(CAM_B);
                              storeB.push(f, preB);
                          });
    if (!ok)
    {
        std::fprintf(stderr, "Could not start the cameras\n");
        camA.stop();
        camB.stop();
        return 1;
    }

    /* ------------------------------------------------------------------
     * 4. Processing objects (all in the main thread)
     * ----------------------------------------------------------------*/
    FrameSync         sync(storeA, storeB, stats);
    CameraCalibration calib(W, H);
    FusionEngine      fusion;
    Viewer            viewer;

    /* Images reused every frame (avoids re-allocating memory) */
    FramePair              pair;
    cv::Mat                bReg;                    /* B warped onto A      */
    cv::Mat                aFloat, bFloat;          /* float copies         */
    std::vector<cv::Mat>   gaussA, gaussB;          /* Gaussian pyramids    */
    std::vector<cv::Mat>   lapA, lapB, lapFused;    /* Laplacian pyramids   */
    cv::Mat                fused8;                  /* final 8-bit image    */

    int waitCount = 0;
    std::printf("Running. Press 'q' or ESC in the window (or Ctrl+C) to quit.\n");

    /* ------------------------------------------------------------------
     * 5. Main loop: runs once per slow-camera frame (~15 Hz)
     * ----------------------------------------------------------------*/
    while (g_running)
    {
        /* Wait (max 100 ms) for a new OV5647 frame and pick the IMX415
         * frame that is closest in time to it.                         */
        PairResult r = sync.next(pair, 100);
        stats.update();

        if (r != PairResult::Ok)
        {
            if (r == PairResult::Timeout && ++waitCount % 30 == 0)
                std::printf("Waiting for camera frames...\n");

            if (!viewer.pollKey())
                break;
            continue;
        }
        waitCount = 0;

        /* ---- Step A: registration (warp B onto A) ---- */
        uint64_t t0 = nowUs();
        calib.registerB(pair.a.image, pair.b.image, bReg);
        uint64_t t1 = nowUs();

        /* ---- Step B: build Gaussian + Laplacian pyramids ---- */
        pair.a.image.convertTo(aFloat, CV_32FC3);
        bReg.convertTo(bFloat, CV_32FC3);

        buildGaussianPyramid(aFloat, cfg::PYRAMID_LEVELS, gaussA);
        buildGaussianPyramid(bFloat, cfg::PYRAMID_LEVELS, gaussB);
        buildLaplacianPyramid(gaussA, lapA);
        buildLaplacianPyramid(gaussB, lapB);
        uint64_t t2 = nowUs();

        /* ---- Step C: fuse the bands ---- */
        fusion.fuse(lapA, lapB, lapFused);
        uint64_t t3 = nowUs();

        /* ---- Step D: rebuild the final image ---- */
        reconstructFromLaplacian(lapFused, fused8);
        uint64_t t4 = nowUs();

        /* ---- Statistics ---- */
        stats.countFused();
        stats.setStageMs(STAGE_REGISTER,    elapsedMs(t0, t1));
        stats.setStageMs(STAGE_PYRAMID,     elapsedMs(t1, t2));
        stats.setStageMs(STAGE_FUSE,        elapsedMs(t2, t3));
        stats.setStageMs(STAGE_RECONSTRUCT, elapsedMs(t3, t4));
        stats.setPairDtMs((double)pair.dtUs / 1000.0);

        /* Latency = from the OLDER sensor timestamp of the pair until now */
        uint64_t oldest = (pair.a.timestampUs < pair.b.timestampUs) ? pair.a.timestampUs
                                                                     : pair.b.timestampUs;
        stats.setLatencyMs(elapsedMs(oldest, t4));

        /* ---- Display ---- */
        if (!viewer.show(pair.a.image, bReg, fused8, lapFused, stats.snapshot()))
            break;
    }

    /* ------------------------------------------------------------------
     * 6. Shutdown: stop the capture threads first, then everything else
     *    is destroyed automatically in reverse order.
     * ----------------------------------------------------------------*/
    std::printf("Stopping...\n");
    camA.stop();
    camB.stop();
    return 0;
}
