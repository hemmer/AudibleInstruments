// Copyright 2019 Emilie Gillet.
//
// Adapted from Mutable Instruments' eurorack/beads/gain_control.{h,cc}.
// The original MIT license notice is retained below. This adapter accepts
// normalized FloatFrame samples; codec scaling is performed by its caller.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// See http://creativecommons.org/licenses/MIT/ for more information.

#ifndef AUDIBLE_BEADS_GAIN_CONTROL_HPP_
#define AUDIBLE_BEADS_GAIN_CONTROL_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "beads/dsp/frame.h"

namespace audible_beads {

class GainControl {
 public:
  enum class SoftClippingMode { Off, On, Dynamic };

  GainControl() = default;

  void init(float sample_rate, SoftClippingMode soft_clipping_mode, bool reset) {
    if (reset) {
      stable_agc_peak_ = agc_peak_ = peak_ = 0.5f;
      gain_ = 1.0f;
      soft_clipping_amount_ = 0.0f;
    }
    soft_clipping_mode_ = soft_clipping_mode;
    ignore_time_ = static_cast<size_t>(sample_rate * kIgnoreTime);
    const bool no_readjust = !reset && time_ && time_ >= active_time_;
    active_time_ = static_cast<size_t>(sample_rate * (kIgnoreTime + kActiveTime));
    time_ = no_readjust ? active_time_ : 0;
    one_hertz_ = 1.0f / sample_rate;
  }

  void restart() { time_ = ignore_time_; }

  void process(bool active, uint8_t gain_setting, bool left_normalization,
               bool right_normalization, const beads::FloatFrame* in,
               beads::FloatFrame* out, size_t size) {
    if (left_normalization && right_normalization) {
      std::fill(out, out + size, beads::FloatFrame{0.0f, 0.0f});
      peak_ = 0.0f;
      agc_peak_ = stable_agc_peak_ = 0.5f;
      gain_ = 1.0f;
      time_ = 0;
      return;
    }

    const float hz = static_cast<float>(size) * one_hertz_;
    if (time_ < ignore_time_) {
      agc_peak_ = 0.5f;
    } else if (time_ < active_time_) {
      slope(agc_peak_, peak_, 10.0f * hz, 2.0f * hz);
      stable_agc_peak_ = agc_peak_;
    } else {
      slope(agc_peak_, peak_, 5.0f * hz,
            (peak_ > stable_agc_peak_ ? 0.5f : 0.01f) * hz);
    }

    if (active) {
      if (time_ > ignore_time_ || peak_ > 0.001f) time_ += size;
      if (time_ >= active_time_) time_ = active_time_;
    }

    auto_gain_ = gain_setting == 0;
    float manual_gain = static_cast<float>(gain_setting) / 255.0f;
    manual_gain *= manual_gain;
    manual_gain *= manual_gain;
    manual_gain = kMinGain + (kMaxGain - kMinGain) * manual_gain;
    manual_gain_ += 0.01f * (manual_gain - manual_gain_);

    const float optimal_gain = std::min(0.9f / std::min(agc_peak_, 0.9f), kMaxGain);
    const float target_gain = auto_gain_ ? optimal_gain : manual_gain_;

    float clipping_target;
    if (soft_clipping_mode_ == SoftClippingMode::Off) {
      clipping_target = 0.0f;
    } else if (soft_clipping_mode_ == SoftClippingMode::On) {
      clipping_target = 1.0f;
    } else {
      clipping_target = 2.0f * (manual_gain_ / optimal_gain - 0.75f);
      clipping_target = std::max(0.0f, std::min(1.0f, clipping_target));
    }

    const float gain_start = gain_;
    const float gain_step = size ? (target_gain - gain_) / static_cast<float>(size) : 0.0f;
    const float clipping_start = soft_clipping_amount_;
    const float clipping_step = size ? (clipping_target - soft_clipping_amount_) /
                                        static_cast<float>(size) : 0.0f;
    float gain_value = gain_start;
    float clipping_value = clipping_start;
    for (size_t i = 0; i < size; ++i) {
      float l = left_normalization ? in[i].r : in[i].l;
      float r = right_normalization ? in[i].l : in[i].r;
      if (active) {
        const float sample_peak = std::max(std::fabs(l), std::fabs(r));
        slope(peak_, sample_peak, 0.1f, 2.0f * one_hertz_);
      }
      gain_value += gain_step;
      l *= gain_value;
      r *= gain_value;
      if (!auto_gain_) {
        if (soft_clipping_mode_ == SoftClippingMode::Off) {
          l = std::max(-1.0f, std::min(1.0f, l));
          r = std::max(-1.0f, std::min(1.0f, r));
        } else {
          clipping_value += clipping_step;
          constexpr float bias = 0.05f;
          const float soft_l = (1.0f - bias) * softClip(l + bias) - bias;
          const float soft_r = (1.0f - bias) * softClip(r + bias) - bias;
          l = crossfade(std::max(-1.0f, std::min(1.0f, l)), soft_l, clipping_value);
          r = crossfade(std::max(-1.0f, std::min(1.0f, r)), soft_r, clipping_value);
        }
      }
      out[i].l = l;
      out[i].r = r;
    }
    gain_ = gain_value;
    if (!auto_gain_ && soft_clipping_mode_ != SoftClippingMode::Off) {
      soft_clipping_amount_ = clipping_value;
    }
  }

  uint8_t vuMeter() const {
    const float intensity = peak_ * gain_;
    return intensity > 1.0f ? 255 : static_cast<uint8_t>(192.0f * intensity * intensity);
  }
  bool adjusting() const { return time_ < active_time_ && time_ > 0 && auto_gain_; }

 private:
  static constexpr float kIgnoreTime = 1.0f;
  static constexpr float kActiveTime = 5.0f;
  static constexpr float kMinGain = 1.0f;
  static constexpr float kMaxGain = 40.0f;

  static void slope(float& out, float in, float positive, float negative) {
    const float error = in - out;
    out += (error > 0.0f ? positive : negative) * error;
  }
  static float crossfade(float a, float b, float fade) { return a + (b - a) * fade; }
  static float softClip(float x) {
    if (x < -3.0f) return -1.0f;
    if (x > 3.0f) return 1.0f;
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
  }

  float peak_ = 0.0f;
  float agc_peak_ = 0.5f;
  float stable_agc_peak_ = 0.5f;
  float gain_ = 1.0f;
  float soft_clipping_amount_ = 0.0f;
  SoftClippingMode soft_clipping_mode_ = SoftClippingMode::Off;
  bool auto_gain_ = true;
  float manual_gain_ = 0.0f;
  float one_hertz_ = 1.0f / 48000.0f;
  size_t time_ = 0;
  size_t ignore_time_ = 48000;
  size_t active_time_ = 288000;
};

}  // namespace audible_beads

#endif  // AUDIBLE_BEADS_GAIN_CONTROL_HPP_
