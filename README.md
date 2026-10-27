# DualCam Fusion Lab — Radxa Rock 4D (RK3576)

Fuses two cameras of different speed into one image, in real time:

| | Camera A | Camera B |
|---|---|---|
| Sensor | IMX415 (fast, up to 60 FPS) | OV5647 (slow, ~15 FPS) |
| Nodes | `/dev/video31`, `/dev/v4l-subdev8` | `/dev/video22`, `/dev/v4l-subdev3` |
| Capture | V4L2 NV12, DMA-BUF | V4L2 NV12, DMA-BUF |

There are **no command-line arguments**. Everything is set in `lib/config/config.h`.

## Build and run

```bash
sudo apt install cmake libopencv-dev librga-dev      # librga is optional (see below)
mkdir build && cd build
cmake ..
make -j4
./dualcam_fusion
```

Press `q` or `ESC` in the window (or Ctrl+C) to quit.

* If CMake prints *"librga not found"* the program still works, but uses a slow OpenCV CPU conversion for the 4K frames. Install `librga-dev` (or point CMake to your librga) to get the hardware path.
* Both cameras must already be linked/configured in the media pipeline the same way as in your other working camera projects. This program only opens the video nodes and sets the NV12 format.

## Folder structure

```
dualcam_fusion/
├── main.cpp                  wires everything together, main loop
├── CMakeLists.txt
├── README.md
└── lib/
    ├── config/               config.h        every tunable number
    ├── dmabuf/               dmabuf.h/.cpp   allocate DMA-BUF from /dev/dma_heap
    ├── camera/               camera.h/.cpp   V4L2 capture, one thread per camera
    ├── preprocess/           image_preprocess.h/.cpp   RGA: crop + resize + NV12->BGR
    ├── sync/                 frame_sync.h/.cpp         frame mailboxes + timestamp pairing
    ├── calibration/          camera_calibration.h/.cpp warp camera B onto camera A
    ├── pyramid/              laplacian_pyramid.h/.cpp  Gaussian + Laplacian pyramids
    ├── fusion/               fusion_engine.h/.cpp      fuse the bands
    ├── reconstruct/          pyramid_reconstruct.h/.cpp rebuild the image
    ├── stats/                pipeline_stats.h/.cpp     FPS, latency, drops, CPU
    └── viewer/               viewer.h/.cpp             OpenCV imshow window
```

## How it works

```
IMX415 thread: DQBUF -> RGA -> mailbox A (8 frames) -> QBUF      (60 fps, never waits)
OV5647 thread: DQBUF -> RGA -> mailbox B (3 frames) -> QBUF      (15 fps, never waits)

main thread, once per new OV5647 frame:
   newest B  +  A frame closest in time   (FrameSync)
   -> register B onto A  -> Gaussian/Laplacian pyramids
   -> fuse bands -> reconstruct -> show
```

**Real-time rules**
* Each camera has its own thread and its own buffers. Neither ever waits for the other or for the main thread.
* A mailbox keeps only the latest few converted frames. Old frames are overwritten.
* The main loop runs at the slow camera's pace. For every new OV5647 frame it picks the IMX415 frame with the closest timestamp. A pair is rejected (and counted as a drop) if the time gap is above `MAX_PAIR_DT_US` or the frame is older than `MAX_FRAME_AGE_US`.
* If fusion is slower than 15 fps, the main loop always jumps to the *newest* OV5647 frame; skipped frames are counted as `skipped`.

**Why RGA runs before pairing (differs slightly from the diagram)**
The capture thread converts every frame with RGA right away and returns the V4L2 buffer immediately. That makes buffer ownership trivial: no buffer is ever held by another thread, so the corruption hazard from your old `camera_return_frame` cannot happen. The cost is that about 45 of the 60 IMX415 frames per second are converted but never used. RGA does this in hardware, so CPU load stays low.

**Fusion rule**
* Detail bands: per pixel, keep the coefficient with the larger activity (sum of absolute values over B, G, R, lightly blurred).
* Base band (smallest level): average of both cameras.
* Pixels of B that fall outside the warped image are filled from A, so the fusion keeps A there.

## Window layout

```
+---------------------+---------------------+
| Camera A (IMX415)   | Camera B registered |
+---------------------+---------------------+
| FUSED               | Fused pyramid bands |
+---------------------+---------------------+
| FPS, latency, pair dt, stage times, CPU, drops |
+------------------------------------------------+
```

## Calibration

Out of the box there is no warp (both images are just scaled to 960x540). To align the cameras:

1. Quick manual tuning: change `B_SCALE`, `B_ROTATE_DEG`, `B_SHIFT_X`, `B_SHIFT_Y` in `config.h`.
2. Proper calibration: create `calibration.yml` in the folder you run from:

```yaml
%YAML:1.0
---
homography_b_to_a: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1., 0., 0.,
           0., 1., 0.,
           0., 0., 1. ]
```

The homography maps pixels of B to pixels of A, both at processing size (960x540). Note that the OV5647 is cropped to 16:9 by RGA before it is scaled.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Red and blue swapped | set `RGA_SWAP_RB = true` in `config.h` |
| `cannot open /dev/dma_heap/system` | the code falls back to `HEAP_CAMERA`; or change `HEAP_PROCESSING` |
| "driver gave WxH" warning | the sensor does not support that size; the program uses what the driver gave |
| Always `Waiting for camera frames...` | check the media links; compare with your working capture project |
| Many `nomatch` drops, large `Pair dt` | the two V4L2 clocks differ; check the timestamp warning at startup, or raise `MAX_PAIR_DT_US` |
| `skipped` keeps growing | fusion is slower than 15 fps: lower `PROC_WIDTH/HEIGHT` or `PYRAMID_LEVELS` |
| Window does not open | OpenCV needs a display (GUI session or `DISPLAY` set) |

## Possible next steps
* RGA warp instead of `warpPerspective` once the homography is fixed.
* Run the A and B pyramid builds in parallel threads.
* Move pyramid / fusion work to NEON-friendly 16-bit integer images or to the GPU.



## Diagram

```
          RADXA ROCK 4D — DUALCAM FUSION LAB
          ==================================

     CAMERA A                              CAMERA B
       IMX415                               OV5647
    4K / up to 60 FPS                    2K / ~15 FPS
         |                                     |
         v                                     v
  +----------------+                    +----------------+
  | V4L2 Capture A |                    | V4L2 Capture B |
  | DMA-BUF export |                    | DMA-BUF export |
  +----------------+                    +----------------+
         |                                     |
         v                                     v
  +----------------+                    +----------------+
  | Frame Metadata |                    | Frame Metadata |
  | Timestamp, ID  |                    | Timestamp, ID  |
  +----------------+                    +----------------+
         |                                     |
         +------------------+------------------+
                            |
                            v
                +------------------------+
                | Frame Pairing /        |
                | Timestamp Management   |
                | Select usable frames   |
                +------------------------+
                            |
                            v
                +--------------------------------+
                | ROCKCHIP RGA                   |
                | Import DMA-BUF where supported |
                | Resize / crop / color convert  |
                | Write to reusable output bufs  |
                +--------------------------------+
                            |
                            v
                +------------------------+
                | Camera Calibration &   |
                | Image Registration     |
                | CPU initially; RGA     |
                | warp if supported      |
                +------------------------+
                            |
                            v
                +------------------------+
                | Common ROI / Validity  |
                | Mask Generation        |
                +------------------------+
                            |
                +-----------+-----------+
                |                       |
                v                       v
       +----------------+      +----------------+
       | Gaussian       |      | Gaussian       |
       | Pyramid A      |      | Pyramid B      |
       +----------------+      +----------------+
                |                       |
                v                       v
       +----------------+      +----------------+
       | Laplacian      |      | Laplacian      |
       | Pyramid A      |      | Pyramid B      |
       +----------------+      +----------------+
                |                       |
                +-----------+-----------+
                            |
                            v
                +------------------------+
                | Multiscale Fusion      |
                | CPU initially          |
                | Fuse corresponding     |
                | pyramid bands          |
                +------------------------+
                            |
                            v
                +------------------------+
                | Fused Laplacian        |
                | Pyramid                |
                +------------------------+
                            |
                            v
                +------------------------+
                | Pyramid Reconstruction |
                | CPU initially          |
                +------------------------+
                            |
                            v
                +--------------------------------+
                | ROCKCHIP RGA                   |
                | Output resize / color convert  |
                | If required and supported       |
                +--------------------------------+
                            |
                            v
                +------------------------+
                | Output / Display Buffer|
                | DMA-BUF sharing where  |
                | supported              |
                +------------------------+
                            |
                            v
                +------------------------+
                | DualCam Viewer         |
                | Camera A | Camera B    |
                | Fused    | Pyramid     |
                | Latency / FPS          |
                +------------------------+


          PERFORMANCE MONITORING
          ----------------------
          Capture -> RGA -> Registration -> Fusion
                     |             |          |
                     +-------------+----------+
                                   |
                                   v
                    FPS / Latency / CPU / Drops
```

License: This project is available for educational and non-commercial use under the Embedded AI Lab Educational Use License. Commercial use requires prior written permission. Please see the LICENSE file for the full terms.
