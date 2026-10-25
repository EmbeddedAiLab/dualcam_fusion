/*
 * viewer.h  -  one OpenCV window with four panels and a status strip.
 *
 *   +----------------------+----------------------+
 *   |  Camera A (IMX415)   |  Camera B (OV5647)   |
 *   +----------------------+----------------------+
 *   |  Fused result        |  Fused Laplacian     |
 *   |                      |  pyramid bands       |
 *   +----------------------+----------------------+
 *   |  FPS / latency / drops / CPU text            |
 *   +---------------------------------------------+
 *
 * Press 'q' or ESC in the window to quit.
 */
#ifndef VIEWER_H
#define VIEWER_H

#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "stats/pipeline_stats.h"

class Viewer
{
public:
    Viewer();
    ~Viewer();

    /* Draw everything and show it. Returns false when the user wants to quit. */
    bool show(const cv::Mat &a,
              const cv::Mat &bRegistered,
              const cv::Mat &fused,
              const std::vector<cv::Mat> &fusedLaplacian,
              const StatsSnapshot &stats);

    /* Keep the window responsive when there is nothing new to show.
     * Returns false when the user wants to quit.                     */
    bool pollKey();

private:
    void drawPanel(const cv::Mat &img, int col, int row, const std::string &title);
    void drawPyramidPanel(const std::vector<cv::Mat> &lap, int col, int row);
    void drawStatusStrip(const StatsSnapshot &s);

    cv::Mat canvas_;
    bool    windowCreated_ = false;
};

#endif /* VIEWER_H */
