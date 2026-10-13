#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <memory>
#include <new>
#include <stdexcept>

#include <speex/speex_resampler.h>
#include "beads/dsp/processor.h"
#include "beads/dsp/synth.h"
#include "beads/dsp/random.h"
#include "GainControl.hpp"

namespace audible_beads {

// Zero the backing storage BEFORE running upstream's empty constructors, as
// static firmware objects are initialized. No live C++ object is overwritten.
template <typename T> std::unique_ptr<T> makeFirmwareObject() {
    static_assert(alignof(T) <= alignof(std::max_align_t));
    void* storage = ::operator new(sizeof(T));
    std::memset(storage, 0, sizeof(T));
    try { return std::unique_ptr<T>(new (storage) T); }
    catch (...) { ::operator delete(storage); throw; }
}

template <typename T, size_t N> class Queue {
    std::array<T, N> data_{};
    size_t read_ = 0, count_ = 0;
public:
    void clear() { read_ = count_ = 0; }
    bool empty() const { return count_ == 0; }
    const T& front() const { return data_[read_]; }
    bool push(const T& value) {
        if (count_ == N) return false;
        data_[(read_ + count_) % N] = value; ++count_; return true;
    }
    T pop() { T value = front(); read_ = (read_ + 1) % N; --count_; return value; }
};

struct Controls {
    // Values after panel/CV conversion: TIME/SIZE/SHAPE bases in [0,1],
    // PITCH base/CV in semitones, CV for other grains = volts / 5,
    // density CV = volts / 5. Pot laws/filtering belong to the Rack control layer.
    beads::Parameters grain{{0.25f, 0.f, 0.f, false}, {0.5f, 0.f, 0.f, false},
                            {0.5f, 0.f, 0.f, false}, {0.f, 0.f, 0.f, false}, false};
    beads::ModulatedParameter density{0.4f, 0.f};
    beads::Processor::MixParameters mix{0.f, 0.5f, 0.f, false};
    // Integers at the external boundary let serialized/UI values be validated
    // before constructing upstream enum objects (invalid enum loads are UB).
    int media = beads::Processor::MEDIA_DIGITAL;
    int seed = beads::SEED_BUTTON_STATE_LOCKED;
    uint8_t gain = 0;
    bool freezeLatch = false;
    bool restartGain = false;  // Rising edge, not a repeatedly asserted command.
};

struct Input {
    float left = 0.f, right = 0.f, seed = 0.f, freeze = 0.f;
    bool leftConnected = false, rightConnected = false, seedConnected = false;
};

struct Output { float left = 0.f, right = 0.f; };

class BeadsEngine {
    // SRC objects are prepared once, never setRates()/destroyed in process().
    // Additional host rates can be prepared here without changing core rates.
    static constexpr std::array<int, 9> hostRates_{{22050, 24000, 32000, 44100,
                                                   48000, 88200, 96000, 176400, 192000}};
    static constexpr std::array<int, 3> engineRates_{{24000, 32000, 48000}};
    struct Converter {
        SpeexResamplerState* state = nullptr;
        Converter() = default;
        ~Converter() { if (state) speex_resampler_destroy(state); }
        Converter(const Converter&) = delete;
        Converter& operator=(const Converter&) = delete;
        void prepare(int from, int to) {
            if (from == to) return;
            int error = 0;
            state = speex_resampler_init(2, from, to, 5, &error);
            if (!state || error) throw std::runtime_error("Beads SRC initialization failed");
        }
        void reset() { if (state) speex_resampler_reset_mem(state); }
        int latency() const { return state ? speex_resampler_get_input_latency(state) : 0; }
        int run(const beads::FloatFrame* in, int count, beads::FloatFrame* out, int capacity) {
            if (!state) { std::copy(in, in + count, out); return count; }
            spx_uint32_t consumed = count, produced = capacity;
            int error = speex_resampler_process_interleaved_float(state,
                reinterpret_cast<const float*>(in), &consumed,
                reinterpret_cast<float*>(out), &produced);
            return error || consumed != static_cast<unsigned>(count) ? -1 : int(produced);
        }
    };
    struct Pair { Converter input, output; };
    struct Core {
        beads::Processor processor;
        beads::Synth synth;
        alignas(4) uint8_t delay[3][256 * 1024];
        uint16_t reverb[32768];
    };
    struct Sample { Input input; Controls controls; };
    struct Event { uint64_t when; bool seed, freeze; };
    struct GateOutput { uint64_t when; bool mode, high; };

    std::unique_ptr<Core> core_;
    std::unique_ptr<std::array<Pair, 27>> converters_;
    Pair* pair_ = nullptr;
    GainControl gain_;
    beads::Random random_;
    uint32_t initialSeed_;
    int hostRate_ = 48000, engineRate_ = 48000, blockSize_ = 12;
    int holdSamples_ = 14;
    uint64_t hostIndex_ = 0, engineIndex_ = 0;
    int position_ = 0, muteBlocks_ = 1000;
    bool synth_ = true, mono_ = false, leftConnected_ = false, rightConnected_ = false;
    beads::Processor::Media media_ = beads::Processor::MEDIA_DIGITAL;
    bool seedHigh_ = false, freezeHigh_ = false, alignedSeed_ = false, alignedFreeze_ = false;
    bool restartHigh_ = false, outputMode_ = false, outputHigh_ = false;
    bool scheduledMode_ = false, scheduledHigh_ = false;
    double noInputSeconds_ = 0.;
    std::array<Sample, 2048> history_{};
    Queue<Event, 4096> events_;
    Queue<GateOutput, 4096> gates_;
    Queue<beads::FloatFrame, 4096> outputs_;
    std::array<beads::FloatFrame, 12> inputBlock_{};
    std::array<stmlib::GateFlags, 12> seedBlock_{};
    std::array<bool, 12> freezeBlock_{};
    bool blockHasSeed_ = false;
    stmlib::GateFlags previousSeed_ = stmlib::GATE_FLAG_LOW;

public:
    struct Stats {
        uint64_t blocks = 0, engineSamples = 0, rises = 0, falls = 0;
        uint64_t freezeRises = 0, freezeFalls = 0, maxEventLateness = 0;
        uint64_t queueErrors = 0, srcErrors = 0, underruns = 0, nonFinite = 0;
    } stats;

    explicit BeadsEngine(uint32_t seed = 0x21u)
        : core_(makeFirmwareObject<Core>()), converters_(new std::array<Pair, 27>),
          random_(seed), initialSeed_(seed) {
        for (size_t h = 0; h < hostRates_.size(); ++h)
            for (size_t e = 0; e < engineRates_.size(); ++e) {
                auto& p = (*converters_)[h * 3 + e];
                p.input.prepare(hostRates_[h], engineRates_[e]);
                p.output.prepare(engineRates_[e], hostRates_[h]);
            }
        uint8_t* buffers[3] = {core_->delay[0], core_->delay[1], core_->delay[2]};
        core_->processor.Init(buffers, core_->reverb, &random_);
        core_->synth.Init(core_->reverb, &random_);
        selectTransport();
        gain_.init(engineRate_, GainControl::SoftClippingMode::Off, true);
    }
    BeadsEngine(const BeadsEngine&) = delete;
    BeadsEngine& operator=(const BeadsEngine&) = delete;

    // Audio-thread reset reuses the existing buffers and prepared converters.
    // Destroy before zeroing storage to reproduce firmware static initialization.
    void reset() {
        Core* storage = core_.get();
        storage->~Core();
        std::memset(static_cast<void*>(storage), 0, sizeof(Core));
        new (storage) Core;
        random_.Seed(initialSeed_);
        uint8_t* buffers[3] = {storage->delay[0], storage->delay[1], storage->delay[2]};
        storage->processor.Init(buffers, storage->reverb, &random_);
        storage->synth.Init(storage->reverb, &random_);
        synth_ = true; mono_ = false;
        leftConnected_ = rightConnected_ = false;
        media_ = beads::Processor::MEDIA_DIGITAL;
        seedHigh_ = freezeHigh_ = restartHigh_ = false;
        noInputSeconds_ = 0.; muteBlocks_ = 1000;
        inputBlock_.fill({}); seedBlock_.fill(stmlib::GATE_FLAG_LOW);
        freezeBlock_.fill(false);
        stats = {};
        selectTransport();
        gain_.init(engineRate_, GainControl::SoftClippingMode::Off, true);
    }

    bool setHostRate(int rate) {
        if (std::find(hostRates_.begin(), hostRates_.end(), rate) == hostRates_.end()) return false;
        if (rate != hostRate_) { hostRate_ = rate; selectTransport(); muteBlocks_ = 1000; }
        return true;
    }
    int hostRate() const { return hostRate_; }
    int engineRate() const { return engineRate_; }
    int blockSize() const { return blockSize_; }
    bool synthMode() const { return synth_; }
    bool mono() const { return mono_; }
    beads::Processor::Media media() const { return media_; }
    uint32_t randomState() const { return random_.state(); }
    uint8_t vuMeter() const { return gain_.vuMeter(); }
    bool adjustingGain() const { return gain_.adjusting(); }
    size_t coreBytes() const { return sizeof(Core); }
    // Nominal SRC + buffering latency, excluding grain/delay/wow/reverb latency.
    double transportLatencySeconds() const {
        return (holdSamples_ + pair_->input.latency()) / double(hostRate_)
             + pair_->output.latency() / double(engineRate_);
    }
    static Output bypass(const Input& in, bool rightOutputConnected) {
        float l = in.leftConnected ? in.left : (in.rightConnected ? in.right : 0.f);
        float r = in.rightConnected ? in.right : l;
        return {rightOutputConnected ? l : (l + r) * 0.5f, r};
    }

    Output process(const Input& in, const Controls& requestedControls, bool rightOutputConnected = true) {
        const Controls controls = sanitize(requestedControls);
        // Hardware comparator thresholds from nominal 3.3 V feedback network.
        seedHigh_ = schmitt(seedHigh_, in.seedConnected ? in.seed : 0.f);
        freezeHigh_ = schmitt(freezeHigh_, in.freeze);
        const bool frozen = freezeHigh_ || controls.freezeLatch;
        const bool anyInput = in.leftConnected || in.rightConnected;
        if (anyInput || frozen || muteBlocks_) noInputSeconds_ = 0.;
        else noInputSeconds_ += 1. / hostRate_;
        bool wantSynth = synth_ ? !anyInput : (!anyInput && noInputSeconds_ >= 10.);
        auto wantMedia = static_cast<beads::Processor::Media>(std::clamp(int(controls.media), 0, 3));
        const bool wantMono = !(in.leftConnected && in.rightConnected);
        if (!frozen && (wantSynth != synth_ || wantMedia != media_ || wantMono != mono_)) {
            synth_ = wantSynth; mono_ = wantMono; media_ = wantMedia;
            // Firmware initializes Synth once and retains its voice/reverb
            // bookkeeping across returns. Both engines share the reverb RAM.
            if (!synth_) core_->processor.Configure(media_, mono_);
            selectTransport(); muteBlocks_ = 1000;
            gain_.init(engineRate_, clipMode(), true);
        }
        if (in.leftConnected != leftConnected_ || in.rightConnected != rightConnected_) {
            leftConnected_ = in.leftConnected; rightConnected_ = in.rightConnected;
            gain_.init(engineRate_, clipMode(), true);
        }
        if (controls.restartGain && !restartHigh_) gain_.restart();
        restartHigh_ = controls.restartGain;
        history_[hostIndex_ % history_.size()] = {in, controls};
        if (seedHigh_ != lastQueuedSeed_ || freezeHigh_ != lastQueuedFreeze_) {
            uint64_t when = uint64_t(std::ceil((hostIndex_ + pair_->input.latency())
                                            * double(engineRate_) / hostRate_));
            if (!events_.push({when, seedHigh_, freezeHigh_})) ++stats.queueErrors;
            else { lastQueuedSeed_ = seedHigh_; lastQueuedFreeze_ = freezeHigh_; }
        }
        beads::FloatFrame input{normalizeAudio(in.left), normalizeAudio(in.right)};
        beads::FloatFrame converted[8]{};
        int count = pair_->input.run(&input, 1, converted, 8);
        if (count < 0) ++stats.srcErrors;
        else for (int i = 0; i < count; ++i) accept(converted[i]);

        Output result{};
        if (hostIndex_ >= uint64_t(holdSamples_)) {
            if (outputs_.empty()) ++stats.underruns;
            else {
                auto frame = outputs_.pop();
                result = {outputVoltage(frame.l), outputVoltage(frame.r)};
            }
        }
        while (!gates_.empty() && gates_.front().when <= hostIndex_) {
            auto gate = gates_.pop(); outputMode_ = gate.mode; outputHigh_ = gate.high;
        }
        if (outputMode_) result.right = outputHigh_ ? 8.f : 0.f;
        if (!rightOutputConnected) result.left = (result.left + result.right) * 0.5f;
        ++hostIndex_;
        return result;
    }

private:
    bool lastQueuedSeed_ = false, lastQueuedFreeze_ = false;
    static float finiteClamp(float v, float low, float high) {
        return std::isfinite(v) ? std::clamp(v, low, high) : 0.f;
    }
    static Controls sanitize(Controls c) {
        for (auto* p : {&c.grain.time, &c.grain.size, &c.grain.shape, &c.grain.pitch}) {
            const bool pitch = p == &c.grain.pitch;
            p->base = finiteClamp(p->base, pitch ? -24.f : 0.f, pitch ? 24.f : 1.f);
            p->cv = finiteClamp(p->cv, pitch ? -60.f : -1.6f, pitch ? 60.f : 1.6f);
            p->attenuverter = finiteClamp(p->attenuverter, -1.f, 1.f);
        }
        c.grain.monophonic_mode = false;  // Upstream experimental feature disabled.
        c.density.base = finiteClamp(c.density.base, 0.f, 1.f);
        c.density.cv = finiteClamp(c.density.cv, -1.f, 1.f);
        c.mix.feedback = finiteClamp(c.mix.feedback, 0.f, 1.f);
        c.mix.dry_wet = finiteClamp(c.mix.dry_wet, 0.f, 1.f);
        c.mix.reverb_amount = finiteClamp(c.mix.reverb_amount, 0.f, 1.f);
        c.seed = std::clamp(c.seed, 0, 2);
        c.media = std::clamp(c.media, 0, 3);
        return c;
    }
    static bool schmitt(bool high, float volts) {
        if (!std::isfinite(volts)) volts = 0.f;
        // Vth = (3.3/10k + comparatorOutput/33k) /
        //       (1/10k + 1/33k + 1/1k), with ideal 0/3.3 V output.
        return high ? volts > 0.2919571f : volts >= 0.3804290f;
    }
    static float normalizeAudio(float v) {
        return std::isfinite(v) ? std::clamp(v / 10.5f, -1.f, 0.999999f) : 0.f;
    }
    static float outputVoltage(float v) {
        return std::isfinite(v) ? -10.f * std::clamp(v, -1.f, 0.999999f) : 0.f;
    }
    GainControl::SoftClippingMode clipMode() const {
        return media_ == beads::Processor::MEDIA_DIRTY_TAPE ? GainControl::SoftClippingMode::On
             : media_ == beads::Processor::MEDIA_CLEAN_TAPE ? GainControl::SoftClippingMode::Dynamic
             : GainControl::SoftClippingMode::Off;
    }
    void selectTransport() {
        engineRate_ = synth_ ? 48000 : int(core_->processor.sample_rate());
        blockSize_ = engineRate_ / 4000;
        size_t h = std::find(hostRates_.begin(), hostRates_.end(), hostRate_) - hostRates_.begin();
        size_t e = std::find(engineRates_.begin(), engineRates_.end(), engineRate_) - engineRates_.begin();
        pair_ = &(*converters_)[h * 3 + e];
        pair_->input.reset(); pair_->output.reset();
        holdSamples_ = int(std::ceil(blockSize_ * double(hostRate_) / engineRate_)) + 2;
        hostIndex_ = engineIndex_ = 0; position_ = 0; blockHasSeed_ = false;
        history_.fill(Sample{}); events_.clear(); gates_.clear(); outputs_.clear();
        previousSeed_ = stmlib::GATE_FLAG_LOW;
        alignedSeed_ = alignedFreeze_ = lastQueuedSeed_ = lastQueuedFreeze_ = false;
        scheduledMode_ = scheduledHigh_ = outputMode_ = outputHigh_ = false;
    }
    Sample alignedControls(uint64_t index) const {
        int64_t host = int64_t(std::floor(index * double(hostRate_) / engineRate_))
                     - pair_->input.latency();
        if (host < 0) return Sample{};
        return history_[uint64_t(host) % history_.size()];
    }
    void accept(const beads::FloatFrame& input) {
        // Serialize coincident edges rather than silently losing a one-host-
        // sample pulse when downsampling. At most one transition per engine
        // sample; excess event bandwidth is bounded and reported on overflow.
        if (!events_.empty() && events_.front().when <= engineIndex_) {
            auto event = events_.pop();
            stats.maxEventLateness = std::max(stats.maxEventLateness, engineIndex_ - event.when);
            if (event.freeze && !alignedFreeze_) ++stats.freezeRises;
            if (!event.freeze && alignedFreeze_) ++stats.freezeFalls;
            alignedSeed_ = event.seed; alignedFreeze_ = event.freeze;
        }
        auto state = alignedControls(engineIndex_);
        inputBlock_[position_] = input;
        seedBlock_[position_] = stmlib::ExtractGateFlags(previousSeed_, alignedSeed_);
        previousSeed_ = seedBlock_[position_];
        if (previousSeed_ & stmlib::GATE_FLAG_RISING) ++stats.rises;
        if (previousSeed_ & stmlib::GATE_FLAG_FALLING) ++stats.falls;
        blockHasSeed_ |= state.input.seedConnected || alignedSeed_;
        freezeBlock_[position_] = alignedFreeze_ || state.controls.freezeLatch;
        ++position_; ++engineIndex_; ++stats.engineSamples;
        if (position_ != blockSize_) return;

        beads::FloatFrame gained[12]{}, rendered[12]{}, resampled[256]{};
        const auto& c = state.controls;
        gain_.process(muteBlocks_ == 0, c.gain, !state.input.leftConnected,
                      !state.input.rightConnected, inputBlock_.data(), gained, blockSize_);
        if (muteBlocks_) --muteBlocks_;
        else {
            const auto seedButton = static_cast<beads::SeedButtonState>(c.seed);
            if (synth_) core_->synth.Render(seedButton, blockHasSeed_ ? seedBlock_.data() : nullptr,
                freezeBlock_.data(), beads::Synth::Resolution(media_), c.density, c.grain,
                c.mix, rendered, blockSize_);
            else core_->processor.Process(seedButton, blockHasSeed_ ? seedBlock_.data() : nullptr,
                freezeBlock_.data(), c.density, c.grain, c.mix, gained, rendered, blockSize_);
        }
        for (int i = 0; i < blockSize_; ++i) {
            if (!std::isfinite(rendered[i].l) || !std::isfinite(rendered[i].r)) {
                ++stats.nonFinite; rendered[i] = 0.f;
            }
            // Firmware postpones the no-input timer while the processor still
            // has audible tails (or an active grain gate), even with no cable.
            if (!synth_ && (std::fabs(rendered[i].l) > 0.01f ||
                           std::fabs(rendered[i].r) > 0.01f)) noInputSeconds_ = 0.;
            bool high = c.mix.output_gate && rendered[i].r < -0.4f;
            if (c.mix.output_gate != scheduledMode_ || high != scheduledHigh_) {
                uint64_t index = engineIndex_ - blockSize_ + i;
                uint64_t when = uint64_t(holdSamples_) + uint64_t(std::ceil(
                    (index + pair_->output.latency()) * double(hostRate_) / engineRate_));
                if (!gates_.push({when, c.mix.output_gate, high})) ++stats.queueErrors;
                else { scheduledMode_ = c.mix.output_gate; scheduledHigh_ = high; }
            }
            if (c.mix.output_gate) rendered[i].r = 0.f;  // Gates are never FIR-filtered.
        }
        int count = pair_->output.run(rendered, blockSize_, resampled, 256);
        if (count < 0) ++stats.srcErrors;
        else for (int i = 0; i < count; ++i)
            if (!outputs_.push(resampled[i])) ++stats.queueErrors;
        ++stats.blocks; position_ = 0; blockHasSeed_ = false;
    }
};

}  // namespace audible_beads
