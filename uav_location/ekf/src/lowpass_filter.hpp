// low_pass_filter.hpp

#pragma once
#include <cmath>
#include <mutex>

class LowPassFilter2ndOrder {
public:
    LowPassFilter2ndOrder(double cutoff_freq)
        : cutoff_freq_(cutoff_freq), initialized_(false) {}

    void setCutoffFrequency(double freq) {
        std::lock_guard<std::mutex> lock(mutex_);
        cutoff_freq_ = freq;
    }

    double update(double input, double dt) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) {
            prev_input1_ = prev_input2_ = input;
            prev_output1_ = prev_output2_ = input;
            initialized_ = true;
            return input;
        }

        computeCoefficients(dt);

        double output = a0_ * input + a1_ * prev_input1_ + a2_ * prev_input2_
                        - b1_ * prev_output1_ - b2_ * prev_output2_;

        // Update history
        prev_input2_ = prev_input1_;
        prev_input1_ = input;

        prev_output2_ = prev_output1_;
        prev_output1_ = output;

        return output;
    }

private:
    void computeCoefficients(double dt) {
        double omega = 2.0 * M_PI * cutoff_freq_;
        double tan_omega = tan(omega * dt / 2.0);
        double tan_omega2 = tan_omega * tan_omega;

        double norm = 1.0 / (1.0 + sqrt(2.0) * tan_omega + tan_omega2);

        a0_ = tan_omega2 * norm;
        a1_ = 2.0 * a0_;
        a2_ = a0_;
        b1_ = 2.0 * (tan_omega2 - 1.0) * norm;
        b2_ = (1.0 - sqrt(2.0) * tan_omega + tan_omega2) * norm;
    }

    double cutoff_freq_;
    bool initialized_;
    std::mutex mutex_;

    // Coefficients
    double a0_, a1_, a2_, b1_, b2_;

    // History
    double prev_input1_, prev_input2_;
    double prev_output1_, prev_output2_;
};
