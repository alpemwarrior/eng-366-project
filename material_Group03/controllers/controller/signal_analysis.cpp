// # Implemented by:

#include "signal_analysis.hpp"
#include "kiss_fft/kiss_fft.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

LightAnalyzer::LightAnalyzer() {
    light_log_file = fopen("data/lights_data.csv", "w");

    if (light_log_file != NULL) {
        fprintf(light_log_file,
                "time,x,y,max_intensity,status,dominant_freq,mean_intensity\n");
        fflush(light_log_file);
    }
}

void LightAnalyzer::update(Pioneer& robot, double current_time, double x, double y) {
    double intensity = robot.get_light_intensity();
    //debugging
    //static int light_counter = 0;
    //if (light_counter % 30 == 0) {
    //  printf("light intensity = %.2f\n", intensity);
    //}
    //light_counter++;

    light_buffer.push_back(intensity);
    if ((int)light_buffer.size() > BUFFER_SIZE) {
        light_buffer.pop_front();
    }

    bool currently_under = (intensity > LIGHT_THRESHOLD);

    if (!was_under_light && currently_under) {
      was_under_light = true;
      current_light = {x, y, intensity};

      current_light_samples.clear();
      current_light_samples.push_back(intensity);
    }
    
    else if (was_under_light && currently_under) {
      current_light_samples.push_back(intensity);

      if (intensity > current_light.max_intensity) {
        current_light.max_intensity = intensity;
        current_light.x = x;
        current_light.y = y;
      }
    }
    else if (was_under_light && !currently_under) {
        was_under_light = false;
        light_id++;

        double dominant_freq = 0.0;
        std::string status = classifyLight(current_light_samples, dominant_freq);

        double mean_intensity = 0.0;
        if (!current_light_samples.empty()) {
          for (double v : current_light_samples) {
              mean_intensity += v;
            }
        mean_intensity /= current_light_samples.size();
    }

    printf("Detected light n°%d, status: %s, location: (%.2f, %.2f) m\n",
           light_id, status.c_str(), current_light.x, current_light.y);

    if (light_log_file != NULL) {
        fprintf(light_log_file,
                "%.4f,%.4f,%.4f,%.4f,%s,%.4f,%.4f\n",
                current_time,
                current_light.x,
                current_light.y,
                current_light.max_intensity,
                status.c_str(),
                dominant_freq,
                mean_intensity);
        fflush(light_log_file);
    }

    current_light = LightDetection();
    current_light_samples.clear();
  }
}

std::string LightAnalyzer::classifyLight(const std::vector<double>& buffer, double& dominant_freq) {
    dominant_freq = 0.0;

    if ((int)buffer.size() < 8) {
        return "unknown";
    }

    // 1. Find max/min
    double max_val = buffer[0];
    double min_val = buffer[0];

    for (double v : buffer) {
        if (v > max_val) max_val = v;
        if (v < min_val) min_val = v;
    }

    // 2. Keep only the bright central region
    std::vector<double> signal;
    double peak_threshold = max_val * 0.85;

    for (double v : buffer) {
        if (v >= peak_threshold) {
            signal.push_back(v);
        }
    }

    // If too few samples near peak, use original buffer
    if ((int)signal.size() < 5) {
        signal = buffer;
    }

    // 3. Compute mean, range, variance
    double mean = 0.0;
    double local_max = signal[0];
    double local_min = signal[0];

    for (double v : signal) {
        mean += v;
        if (v > local_max) local_max = v;
        if (v < local_min) local_min = v;
    }

    mean /= signal.size();

    double variance = 0.0;

    for (double v : signal) {
        double diff = v - mean;
        variance += diff * diff;
    }

    variance /= signal.size();

    double range = local_max - local_min;

    // 4. Good light- stable near peak
    if (range < 0.30 || variance < 0.02) {
        return "good";
    }

    // 5. Defect- repeated strong drops close to dark level
    int strong_drops = 0;
    double defect_low_threshold = 1.25;  // close to your dark value ≈ 1.00

    for (int i = 1; i < (int)signal.size(); i++) {
        if (signal[i - 1] > mean && signal[i] < defect_low_threshold) {
            strong_drops++;
        }
    }

    if (strong_drops >= 2) {
        return "defect";
    }

    // 6. Flicker- many fast oscillations
    int direction_changes = 0;

    for (int i = 2; i < (int)signal.size(); i++) {
        double diff1 = signal[i - 1] - signal[i - 2];
        double diff2 = signal[i] - signal[i - 1];

        if ((diff1 > 0 && diff2 < 0) || (diff1 < 0 && diff2 > 0)) {
            if (std::fabs(diff1) > 0.05 && std::fabs(diff2) > 0.05) {
                direction_changes++;
            }
        }
    }

    if (direction_changes >= 4 && range >= 0.30) {
        return "flicker";
    }

    // 7. If unsure, avoid false flicker
    return "good";
}

void LightAnalyzer::computeFFT(const std::vector<double>& signal, std::vector<double>& magnitudes) {
    int n = (int)signal.size();

    if (n <= 0) {
        magnitudes.clear();
        return;
    }

    double mean = 0.0;
    for (double v : signal) {
        mean += v;
    }
    mean /= n;

    kiss_fft_cfg cfg = kiss_fft_alloc(n, 0, NULL, NULL);
    if (cfg == NULL) {
        magnitudes.clear();
        return;
    }

    kiss_fft_cpx* in = new kiss_fft_cpx[n];
    kiss_fft_cpx* out = new kiss_fft_cpx[n];

    for (int i = 0; i < n; ++i) {
        in[i].r = signal[i] - mean;
        in[i].i = 0.0;
    }

    kiss_fft(cfg, in, out);

    magnitudes.resize(n);
    for (int i = 0; i < n; ++i) {
        magnitudes[i] = std::sqrt(out[i].r * out[i].r + out[i].i * out[i].i);
    }

    delete[] in;
    delete[] out;
    free(cfg);
}

//void LightAnalyzer::reset() {
//    light_buffer.clear();
//    current_light = LightDetection();
//    light_id = 0;
//    was_under_light = false;
//}