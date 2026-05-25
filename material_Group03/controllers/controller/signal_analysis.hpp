//# Implemented by:

#ifndef SIGNAL_ANALYSIS_H
#define SIGNAL_ANALYSIS_H

#include <vector>
#include <deque>
#include <string>
#include "pioneer_interface/pioneer_interface.hpp"

struct LightDetection {
    double x = 0.0;
    double y = 0.0;
    double max_intensity = 0.0;
};

class LightAnalyzer {
public:
    LightAnalyzer();
    void update(Pioneer& robot, double current_time, double x, double y);
    void reset();

private:
    std::deque<double> light_buffer;
    std::vector<double> current_light_samples;
    LightDetection current_light;
    int light_id = 0;
    bool was_under_light = false;

    // parameters
    const int BUFFER_SIZE = 256;
    const double LIGHT_THRESHOLD = 1.5;
    const double GOOD_VARIANCE_THRESHOLD = 0.004;
    const double DEFECT_FREQ_THRESHOLD = 2.0;
    const double SAMPLING_RATE = 31.25;

    std::string classifyLight(const std::vector<double>& buffer, double& dominant_freq);
    void computeFFT(const std::vector<double>& signal, std::vector<double>& magnitudes);

    FILE* light_log_file = NULL;
};

#endif