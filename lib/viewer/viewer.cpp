#include "viewer/viewer.h"

#include <cstdio>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
const char *WINDOW_NAME = "DualCam Fusion Lab";

const int PANEL_W = 640;
const int PANEL_H = 360;
const int STRIP_H = 70;

const cv::Scalar WHITE(255, 255, 255);
const cv::Scalar GREEN(0, 255, 0);

/* Write text with a dark outline so it is readable on any image */
void putLabel(cv::Mat &img, const std::string &text, cv::Point pos,
              double scale, cv::Scalar colour)
{
    cv::putText(img, text, pos, cv::FONT_HERSHEY_SIMPLEX, scale,
                cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(img, text, pos, cv::FONT_HERSHEY_SIMPLEX, scale,
                colour, 1, cv::LINE_AA);
}
}

Viewer::Viewer()
{
    canvas_ = cv::Mat(PANEL_H * 2 + STRIP_H, PANEL_W * 2, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    windowCreated_ = true;
}

Viewer::~Viewer()
{
    if (windowCreated_)
        cv::destroyWindow(WINDOW_NAME);
}

/* Scale `img` into one grid cell and print a title in the corner */
void Viewer::drawPanel(const cv::Mat &img, int col, int row, const std::string &title)
{
    cv::Mat roi = canvas_(cv::Rect(col * PANEL_W, row * PANEL_H, PANEL_W, PANEL_H));

    if (img.empty())
        roi.setTo(cv::Scalar(30, 30, 30));
    else
        cv::resize(img, roi, roi.size(), 0, 0, cv::INTER_AREA);

    putLabel(roi, title, cv::Point(10, 25), 0.7, GREEN);
}

/* Show every pyramid band as a small tile (3 columns x 2 rows).
 * Detail bands are drawn as grey images centred on 128 with 4x gain;
 * the last tile is the base (low-frequency) image.                  */
void Viewer::drawPyramidPanel(const std::vector<cv::Mat> &lap, int col, int row)
{
    cv::Mat roi = canvas_(cv::Rect(col * PANEL_W, row * PANEL_H, PANEL_W, PANEL_H));
    roi.setTo(cv::Scalar(30, 30, 30));

    const int cols  = 3;
    const int tileW = PANEL_W / cols;
    const int tileH = PANEL_H / 2;

    for (size_t i = 0; i < lap.size() && i < 6; i++)
    {
        cv::Mat vis;
        if (i + 1 == lap.size())
        {
            lap[i].convertTo(vis, CV_8UC3);                       /* base */
        }
        else
        {
            cv::Mat grey, grey8;
            cv::cvtColor(lap[i], grey, cv::COLOR_BGR2GRAY);
            grey.convertTo(grey8, CV_8U, 4.0, 128.0);             /* gain 4, offset 128 */
            cv::cvtColor(grey8, vis, cv::COLOR_GRAY2BGR);
        }

        cv::Mat tile = roi(cv::Rect((int)(i % cols) * tileW,
                                    (int)(i / cols) * tileH,
                                    tileW, tileH));
        cv::resize(vis, tile, tile.size(), 0, 0, cv::INTER_AREA);

        char name[32];
        if (i + 1 == lap.size())
            std::snprintf(name, sizeof(name), "base (L%zu)", i);
        else
            std::snprintf(name, sizeof(name), "band L%zu", i);
        putLabel(tile, name, cv::Point(6, 18), 0.5, GREEN);
    }

    putLabel(roi, "Fused pyramid", cv::Point(10, PANEL_H - 10), 0.6, WHITE);
}

void Viewer::drawStatusStrip(const StatsSnapshot &s)
{
    cv::Mat strip = canvas_(cv::Rect(0, PANEL_H * 2, PANEL_W * 2, STRIP_H));
    strip.setTo(cv::Scalar(20, 20, 20));

    char l1[200], l2[200];

    std::snprintf(l1, sizeof(l1),
        "FPS  IMX415 %.1f | OV5647 %.1f | Fused %.1f      Latency %.0f ms      Pair dt %+.1f ms",
        s.fpsA, s.fpsB, s.fpsFused, s.latencyMs, s.pairDtMs);

    std::snprintf(l2, sizeof(l2),
        "Stage ms: reg %.1f  pyr %.1f  fuse %.1f  recon %.1f      CPU %.0f%%      "
        "Drops: skipped %llu  nomatch %llu  old %llu",
        s.stageMs[STAGE_REGISTER], s.stageMs[STAGE_PYRAMID],
        s.stageMs[STAGE_FUSE], s.stageMs[STAGE_RECONSTRUCT], s.cpuPercent,
        (unsigned long long)s.drops[DROP_SKIPPED_B],
        (unsigned long long)s.drops[DROP_NO_MATCH],
        (unsigned long long)s.drops[DROP_TOO_OLD]);

    cv::putText(strip, l1, cv::Point(10, 28), cv::FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1, cv::LINE_AA);
    cv::putText(strip, l2, cv::Point(10, 56), cv::FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv::LINE_AA);
}

bool Viewer::show(const cv::Mat &a,
                  const cv::Mat &bRegistered,
                  const cv::Mat &fused,
                  const std::vector<cv::Mat> &fusedLaplacian,
                  const StatsSnapshot &stats)
{
    drawPanel(a,           0, 0, "Camera A - IMX415");
    drawPanel(bRegistered, 1, 0, "Camera B - OV5647 (registered)");
    drawPanel(fused,       0, 1, "FUSED");
    drawPyramidPanel(fusedLaplacian, 1, 1);
    drawStatusStrip(stats);

    cv::imshow(WINDOW_NAME, canvas_);
    return pollKey();
}

bool Viewer::pollKey()
{
    int key = cv::waitKey(1);
    return !(key == 27 || key == 'q' || key == 'Q');
}
