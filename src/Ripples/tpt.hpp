// Economy Ripples core adapted from Vostok Instruments' AtlasTptEngine4.
// Original Ripples emulation Copyright (C) 2020 Tyler Coy.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "ripples.hpp"
#include "ripples_v2.hpp"

namespace ripples_tpt {
using namespace rack;
// Recompute the two dynamic frequency constants here to avoid initialization
// order dependence on the reference headers' namespace-scope variables.
struct Original {
    using Frame = ripples::RipplesEngine::Frame;
    static constexpr bool v2 = false;
    inline static const float kFreqKnobVoltage = std::log2(20000.f / 20.f);
    inline static const float kFreqKnobMax = ripples::kFreqKnobMax;
    inline static const float kFreqAmpR = 0.033f * 20.f * std::log10(2.f) * 100e3f;
    inline static const float kFreqAmpC = ripples::kFreqAmpC;
    inline static const float kResAmpR = ripples::kResAmpR;
    inline static const float kResAmpC = ripples::kResAmpC;
    inline static const float kResInputR = ripples::kResInputR;
    inline static const float kResKnobV = ripples::kResKnobV;
    inline static const float kResKnobR = ripples::kResKnobR;
    inline static const float kGainAmpR = ripples::kGainAmpR;
    inline static const float kGainAmpC = ripples::kGainAmpC;
    inline static const float kGainInputR = ripples::kGainInputR;
    inline static const float kGainNormalV = ripples::kGainNormalV;
    inline static const float kGainNormalR = ripples::kGainNormalR;
    inline static const float kFeedforwardR = ripples::kFeedforwardR;
    inline static const float kFeedforwardC = ripples::kFeedforwardC;
    inline static const float kFeedforwardGain = ripples::kFeedforwardGain;
    inline static const float kFilterInputGain = ripples::kFilterInputGain;
    inline static const float kFilterCellR = ripples::kFilterCellR;
    inline static const float kFeedbackGain = ripples::kFeedbackGain;
    inline static const float kVtoICollectorVSat = ripples::kVtoICollectorVSat;
    inline static const float kVCAInputR = ripples::kVCAInputR;
    inline static const float kVCAInputC = ripples::kVCAInputC;
    inline static const float kVCAInputGain = ripples::kVCAInputGain;
    inline static const float kVCAOutputR = ripples::kVCAOutputR;
    inline static const float kBP2Gain = ripples::kBP2Gain;
    inline static const float kLP2Gain = ripples::kLP2Gain;
    inline static const float kLP4Gain = ripples::kLP4Gain;
};
struct V2 {
    using Frame = ripples_2020::RipplesEngine::Frame;
    static constexpr bool v2 = true;
    inline static const float kFreqKnobVoltage = std::log2(20000.f / 20.f);
    inline static const float kFreqKnobMax = ripples_2020::kFreqKnobMax;
    inline static const float kFreqAmpR = 0.033f * 20.f * std::log10(2.f) * 100e3f;
    inline static const float kFreqAmpC = ripples_2020::kFreqAmpC;
    inline static const float kResAmpR = ripples_2020::kResAmpR;
    inline static const float kResAmpC = ripples_2020::kResAmpC;
    inline static const float kResInputR = ripples_2020::kResInputR;
    inline static const float kResKnobV = ripples_2020::kResKnobV;
    inline static const float kResKnobR = ripples_2020::kResKnobR;
    inline static const float kGainAmpR = ripples_2020::kGainAmpR;
    inline static const float kGainAmpC = ripples_2020::kGainAmpC;
    inline static const float kGainInputR = ripples_2020::kGainInputR;
    inline static const float kGainNormalV = ripples_2020::kGainNormalV;
    inline static const float kGainNormalR = ripples_2020::kGainNormalR;
    inline static const float kFeedforwardR = ripples_2020::kFeedforwardR;
    inline static const float kFeedforwardC = ripples_2020::kFeedforwardC;
    inline static const float kFeedforwardGain = ripples_2020::kFeedforwardGain;
    inline static const float kFilterInputGain = ripples_2020::kFilterInputGain;
    inline static const float kFilterCellR = ripples_2020::kFilterCellR;
    inline static const float kFeedbackGain = ripples_2020::kFeedbackGain;
    inline static const float kVtoICollectorVSat = ripples_2020::kVtoICollectorVSat;
    inline static const float kVCAInputR = ripples_2020::kVCAInputR;
    inline static const float kVCAInputC = ripples_2020::kVCAInputC;
    inline static const float kVCAInputGain = ripples_2020::kVCAInputGain;
    inline static const float kVCAOutputR = ripples_2020::kVCAOutputR;
    inline static const float kBP2Gain = ripples_2020::kBP2Gain;
    inline static const float kHP2Gain = ripples_2020::kHP2Gain;
    inline static const float kBP4Gain = ripples_2020::kBP4Gain;
};

// Four independent voices per SIMD group. The cascade is affine in its
// input, so two Newton iterations close only the scalar OTA feedback loop.
template <typename C>
struct Engine {
    using float_4 = simd::float_4;
    using Frame = typename C::Frame;
    float sampleTime = 1.f / 48000.f;
    float frequencySmoothing, resonanceSmoothing, gainSmoothing, feedforwardSmoothing;
    float_4 stageState[4]{};
    float_4 smoothedVOct{}, smoothedResCurrent{}, smoothedGainCurrent{}, feedforwardLowpass{};
    dsp::TRCFilter<float_4> vcaHighpass;

    Engine() { setSampleRate(48000.f); }
    void resetLane(int lane) {
        for (auto& state : stageState) state[lane] = 0.f;
        smoothedVOct[lane] = smoothedResCurrent[lane] = smoothedGainCurrent[lane] = feedforwardLowpass[lane] = 0.f;
        vcaHighpass.xstate[0][lane] = vcaHighpass.ystate[0][lane] = 0.f;
    }
    void setSampleRate(float sampleRate) {
        sampleTime = 1.f / std::max(sampleRate, 1.f);
        frequencySmoothing = std::exp(-sampleTime / (C::kFreqAmpR * C::kFreqAmpC));
        resonanceSmoothing = std::exp(-sampleTime / (C::kResAmpR * C::kResAmpC));
        gainSmoothing = std::exp(-sampleTime / (C::kGainAmpR * C::kGainAmpC));
        feedforwardSmoothing = std::exp(-sampleTime / (C::kFeedforwardR * C::kFeedforwardC));
        vcaHighpass.setCutoffFreq(sampleTime / (2.f * M_PI * C::kVCAInputR * C::kVCAInputC));
        for (int lane = 0; lane < 4; ++lane) resetLane(lane);
    }
    void process(Frame* frames, int count) {
        float_4 input = 0.f, vOct = 0.f, resCv = 0.f, resKnob = 0.f, gainCv = 0.f;
        float_4 gainResistance = C::kGainInputR;
        for (int lane = 0; lane < count; ++lane) {
            auto& f = frames[lane];
            input[lane] = f.input + 1e-6f * (random::uniform() - 0.5f);
            if constexpr (C::v2)
                input[lane] = ripples_2020::clipFactor * std::tanh(input[lane] / ripples_2020::clipFactor) + f.input2;
            vOct[lane] = std::min((f.freq_knob - 1.f) * C::kFreqKnobVoltage + f.freq_cv + f.fm_cv * f.fm_knob, 0.f);
            resCv[lane] = f.res_cv;
            resKnob[lane] = f.res_knob;
            gainCv[lane] = f.gain_cv_present ? f.gain_cv : C::kGainNormalV;
            gainResistance[lane] = C::kGainInputR + (f.gain_cv_present ? 0.f : C::kGainNormalR);
        }
        smoothedVOct = frequencySmoothing * smoothedVOct + (1.f - frequencySmoothing) * vOct;
        const float_4 resonanceCurrent = VtoIConverter(C::kResAmpR, resCv, C::kResInputR, resKnob * C::kResKnobV, C::kResKnobR);
        smoothedResCurrent = resonanceSmoothing * smoothedResCurrent + (1.f - resonanceSmoothing) * resonanceCurrent;
        const float_4 gainCurrent = VtoIConverter(C::kGainAmpR, gainCv, gainResistance, 0.f, 1e12f);
        smoothedGainCurrent = gainSmoothing * smoothedGainCurrent + (1.f - gainSmoothing) * gainCurrent;
        feedforwardLowpass = feedforwardSmoothing * feedforwardLowpass + (1.f - feedforwardSmoothing) * input;
        const float_4 feedforward = (input - feedforwardLowpass) * C::kFeedforwardGain * (C::v2 ? C::kFilterInputGain : 1.f);

        // Keep tan() away from its Nyquist singularity at low host rates.
        const float_4 cutoff = simd::fmin(C::kFreqKnobMax * simd::exp(smoothedVOct * std::log(2.f)), 0.45f / sampleTime);
        const float_4 g = simd::tan(float(M_PI) * cutoff * sampleTime);
        const float_4 a = g / (1.f + g);
        const float_4 b = 1.f / (1.f + g);
        float_4 alpha = 1.f, beta = 0.f;
        for (int i = 0; i < 4; ++i) {
            alpha = -a * alpha;
            beta = -a * beta - b * stageState[i];
        }
        const float_4 baseInput = input * C::kFilterInputGain;
        float_4 filterInput = baseInput;
        for (int iteration = 0; iteration < 2; ++iteration) {
            const float_4 feedback = (alpha * filterInput + beta) * C::kFeedbackGain;
            const float_4 residual = filterInput - baseInput - C::kFilterCellR * OTA(feedforward, feedback, smoothedResCurrent);
            const float_4 derivative = 1.f + C::kFilterCellR * otaDerivative(feedforward, feedback, smoothedResCurrent) * C::kFeedbackGain * alpha;
            filterInput -= residual / simd::fmax(derivative, 0.25f);
        }
        float_4 taps[4];
        float_4 stageInput = filterInput;
        for (int i = 0; i < 4; ++i) {
            const float_4 z = a * stageInput + b * stageState[i];
            taps[i] = -z;
            stageState[i] = 2.f * z - stageState[i];
            stageInput = taps[i];
        }
        float_4 lp = taps[3];
        if constexpr (C::v2) {
            for (int lane = 0; lane < count; ++lane)
                if (frames[lane].slope == 0) lp[lane] = taps[1][lane];
        }
        vcaHighpass.process(lp);
        const float_4 lpvca = C::kVCAOutputR * OTA(vcaHighpass.highpass() * C::kVCAInputGain, 0.f, smoothedGainCurrent);
        for (int lane = 0; lane < count; ++lane) {
            auto& f = frames[lane];
            if constexpr (C::v2) {
                f.hp = ((filterInput + 2.f * taps[0] + taps[1]) * C::kHP2Gain)[lane];
                f.bp = f.slope == 0 ? ((taps[0] + taps[1]) * C::kBP2Gain)[lane] : ((taps[1] + 2.f * taps[2] + taps[3]) * C::kBP4Gain)[lane];
                f.lp = 0.f;
                f.lpvca = lpvca[lane];
            } else {
                f.bp2 = ((taps[0] + taps[1]) * C::kBP2Gain)[lane];
                f.lp2 = (taps[1] * C::kLP2Gain)[lane];
                f.lp4 = (taps[3] * C::kLP4Gain)[lane];
                f.lp4vca = lpvca[lane];
            }
        }
        for (int lane = count; lane < 4; ++lane) resetLane(lane);
    }
private:
    static float_4 VtoIConverter(float rfb, float_4 vc, float_4 rc, float_4 vp, float rp) {
        const float_4 vnom = -(vc * rfb / rc + vp * rfb / rp);
        const float_4 vout = simd::fmax(vnom, C::kVtoICollectorVSat);
        const float nrc = rp * rfb;
        const float_4 nrp = rc * rfb;
        const float_4 nrfb = rc * rp;
        const float_4 vneg = (vc * nrc + vp * nrp + vout * nrfb) / (nrc + nrp + nrfb);
        return simd::fmax((vneg - vout) / rfb, 0.f);
    }

    static float_4 OTA(float_4 vp, float_4 vn, float_4 iAbc) {
        constexpr float vt = 8.617333262145e-5f * (40.f + 273.15f);
        constexpr float zlim = 3.4641016151377544f;
        const float_4 z = simd::clamp((vp - vn) / (2.f * vt), -zlim, zlim);
        const float_4 z2 = z * z;
        const float_4 q = 12.f + z2;
        const float_4 denominator = 36.f * z2 + q * q;
        return iAbc * (12.f * z * q / denominator);
    }

    static float_4 otaDerivative(float_4 vp, float_4 vn, float_4 iAbc) {
        constexpr float vt = 8.617333262145e-5f * (40.f + 273.15f);
        constexpr float zlim = 3.4641016151377544f;
        const float_4 rawZ = (vp - vn) / (2.f * vt);
        const float_4 z = simd::clamp(rawZ, -zlim, zlim);
        const float_4 z2 = z * z;
        const float_4 q = 12.f + z2;
        const float_4 n = 12.f * z * q;
        const float_4 dn = 12.f * (12.f + 3.f * z2);
        const float_4 d = 36.f * z2 + q * q;
        const float_4 dd = z * (120.f + 4.f * z2);
        const float_4 dpdz = (dn * d - n * dd) / (d * d);
        const float_4 derivative = iAbc * dpdz / (2.f * vt);
        const float_4 saturated = (rawZ <= -zlim) | (rawZ >= zlim);
        return simd::ifelse(saturated, float_4::zero(), derivative);
    }

};
} // namespace ripples_tpt
