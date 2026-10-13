#pragma once

#include <algorithm>
#include <cmath>

#include "beads/dsp/processor.h"
#include "beads/dsp/trigger_generator.h"
#include "beads/dsp/randomized_parameter.h"

namespace audible_beads {

// Rack-side translation of the Beads CV/pot laws. The four modulation pots
// have a signed setting: without a cable they set internal randomness; with a
// cable CW attenuates CV and CCW sets CV-controlled randomization. Pitch uses the firmware's stepped
// detune plateaus, while continuous CV remains semitone-scaled.
struct ControlLayer {
    static float finite(float x, float fallback = 0.f) {
        return std::isfinite(x) ? x : fallback;
    }
    static float clamp01(float x) { return std::clamp(finite(x), 0.f, 1.f); }
    static float attenuverter(float x) { return std::clamp(finite(x), -1.f, 1.f); }
    static float attenuverterPotLaw(float x) {
        float v = std::clamp(finite(x), -1.f, 1.f) * .5f;
        return (2.f / .47f) * v * std::max(std::fabs(v) - .03f, 0.f);
    }

    static float pitchPot(float x) {
        x = clamp01(x);
        float d = (2.f * x - 1.f) * 6.f;
        float sign = d < 0.f ? -1.f : 1.f;
        d = std::fabs(d);
        constexpr float intervals[] = {0.f, 1.f, 3.f, 4.f, 7.f, 12.f, 24.f};
        int i = std::min(int(d), 5);
        float f = std::clamp((d - float(i) - 0.33333333f) * 1.5f, 0.f, 1.f);
        float smooth = f * f * (3.f - 2.f * f);
        return sign * (intervals[i] + (intervals[i + 1] - intervals[i]) * smooth);
    }

    // One-pole filtering mirrors the firmware's 4 kHz control-rate readout.
    struct Smoother {
        float value = 0.f;
        bool initialized = false;
        void process(float input, float coefficient) {
            input = finite(input);
            if (!initialized) { value = input; initialized = true; }
            else value += (input - value) * coefficient;
        }
    };

    static constexpr int MEDIA_DIRTY_TAPE = beads::Processor::MEDIA_DIRTY_TAPE;
    static constexpr int MEDIA_CLEAN_TAPE = beads::Processor::MEDIA_CLEAN_TAPE;
    static constexpr int MEDIA_VINTAGE_DIGITAL = beads::Processor::MEDIA_VINTAGE_DIGITAL;
    static constexpr int MEDIA_DIGITAL = beads::Processor::MEDIA_DIGITAL;
    static constexpr int SEED_OFF = beads::SEED_BUTTON_STATE_OFF;
    static constexpr int SEED_ON = beads::SEED_BUTTON_STATE_ON;
    static constexpr int SEED_LOCKED = beads::SEED_BUTTON_STATE_LOCKED;
};

} // namespace audible_beads
