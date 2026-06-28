#pragma once
#include <atomic>
#include <limits>

namespace FrontLidar {
    // Front-center sector min range (±30° horizontal, robot frame).
    // Written by SensorProducerWebots::pushLidar, read by main loop before sendDriveCommand.
    inline std::atomic<float>  g_min_range_m{std::numeric_limits<float>::infinity()};
    inline std::atomic<double> g_min_range_ts{-1.0};
}
