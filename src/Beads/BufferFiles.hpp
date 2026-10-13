#pragma once
#include "BeadsEngine.hpp"
#include "dr_wav.h"
#include <fstream>
#include <vector>
#if defined(_WIN32)
#include <filesystem>
#endif

namespace audible_beads {

inline auto bufferFilePath(const std::string& path) {
#if defined(_WIN32)
    return std::filesystem::u8path(path);
#else
    return path;
#endif
}

inline uint32_t bufferChecksum(const FrozenBuffer& buffer) {
    uint32_t hash = 2166136261u;
    const uint32_t metadata[] = {uint32_t(buffer.media), uint32_t(buffer.mono),
        uint32_t(buffer.state.write_ptr), uint32_t(buffer.state.duration),
        uint32_t(buffer.state.tail_write_ptr), uint32_t(buffer.state.tail_xfade_duration),
        uint32_t(buffer.state.written_since_last_frozen)};
    for (uint32_t word : metadata)
        for (int i = 0; i < 4; ++i) hash = (hash ^ uint8_t(word >> (8 * i))) * 16777619u;
    for (const auto& bank : buffer.banks)
        for (uint8_t byte : bank) hash = (hash ^ byte) * 16777619u;
    return hash;
}

// A versioned little-endian container preserves the original compressed bytes,
// freeze tail, duration and write head; no re-encoding on a Rack restart.
inline bool writeFrozenFile(const std::string& path, const FrozenBuffer& buffer) {
    if (!buffer.valid()) return false;
    std::ofstream file(bufferFilePath(path), std::ios::binary | std::ios::trunc);
    file.write("BEADBUF1", 8);
    auto word = [&](uint32_t value) {
        char bytes[4];
        for (int i = 0; i < 4; ++i) bytes[i] = char(value >> (8 * i));
        file.write(bytes, 4);
    };
    word(buffer.media); word(buffer.mono); word(buffer.state.write_ptr);
    word(buffer.state.duration); word(buffer.state.tail_write_ptr);
    word(buffer.state.tail_xfade_duration); word(buffer.state.written_since_last_frozen);
    word(bufferChecksum(buffer));
    for (const auto& bank : buffer.banks)
        file.write(reinterpret_cast<const char*>(bank.data()), bank.size());
    file.close();
    return bool(file);
}

inline bool readFrozenFile(const std::string& path, FrozenBuffer& buffer) {
    std::ifstream file(bufferFilePath(path), std::ios::binary);
    char magic[8]; file.read(magic, 8);
    if (!file || std::memcmp(magic, "BEADBUF1", 8)) return false;
    auto word = [&]() {
        unsigned char bytes[4]{};
        file.read(reinterpret_cast<char*>(bytes), 4);
        return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8)
            | (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
    };
    uint32_t values[8];
    for (auto& value : values) value = word();
    if (!file || values[0] > 3 || values[1] > 1) return false;
    for (int i = 2; i < 7; ++i) if (values[i] > uint32_t(INT32_MAX)) return false;
    buffer.media = int(values[0]); buffer.mono = values[1];
    buffer.state = {int(values[2]), int(values[3]), int(values[4]), int(values[5]), int(values[6]), true};
    if (!buffer.valid()) return false;
    for (auto& bank : buffer.banks)
        file.read(reinterpret_cast<char*>(bank.data()), bank.size());
    if (!file || file.peek() != std::char_traits<char>::eof()) return false;
    if (bufferChecksum(buffer) != values[7]) return false;
    // The codec data accepts all bit patterns; the float freeze tail does not.
    for (size_t i = FrozenBuffer::bankBytes - 256 * sizeof(beads::FloatFrame);
         i < FrozenBuffer::bankBytes; i += sizeof(beads::FloatFrame)) {
        beads::FloatFrame frame;
        std::memcpy(&frame, buffer.banks[2].data() + i, sizeof(frame));
        if (!std::isfinite(frame.l) || !std::isfinite(frame.r)) return false;
    }
    return true;
}

inline bool writeBufferWav(const std::string& path, const FrozenBuffer& buffer) {
    if (!buffer.valid()) return false;
    std::vector<beads::FloatFrame> frames(buffer.state.duration);
    int count = BeadsEngine::exportFrozenBuffer(buffer, frames.data(), int(frames.size()));
    if (!count) return false;
    std::vector<float> samples(size_t(count) * (buffer.mono ? 1 : 2));
    for (int i = 0; i < count; ++i) {
        samples[size_t(i) * (buffer.mono ? 1 : 2)] = frames[i].l;
        if (!buffer.mono) samples[2 * i + 1] = frames[i].r;
    }
    drwav_data_format format{};
    format.container = drwav_container_riff; format.format = DR_WAVE_FORMAT_IEEE_FLOAT;
    format.channels = buffer.mono ? 1 : 2;
    format.sampleRate = FrozenBuffer::sampleRate(buffer.media); format.bitsPerSample = 32;
    drwav wav;
#if defined(_WIN32)
    if (!drwav_init_file_write_w(&wav, bufferFilePath(path).wstring().c_str(), &format, nullptr)) return false;
#else
    if (!drwav_init_file_write(&wav, path.c_str(), &format, nullptr)) return false;
#endif
    const auto written = drwav_write_pcm_frames(&wav, count, samples.data());
    const auto result = drwav_uninit(&wav);
    return written == unsigned(count) && result == DRWAV_SUCCESS;
}

inline bool readBufferWav(const std::string& path, int media, FrozenBuffer& buffer) {
    if (media < 0 || media > 3) return false;
    drwav wav;
#if defined(_WIN32)
    if (!drwav_init_file_w(&wav, bufferFilePath(path).wstring().c_str(), nullptr)) return false;
#else
    if (!drwav_init_file(&wav, path.c_str(), nullptr)) return false;
#endif
    const unsigned channels = wav.channels, sourceRate = wav.sampleRate;
    const bool valid = (channels == 1 || channels == 2) && sourceRate >= 8000
        && sourceRate <= 192000 && wav.totalPCMFrameCount > 0;
    if (!valid) { drwav_uninit(&wav); return false; }
    const int capacity = FrozenBuffer::capacity(media, channels == 1);
    const int targetRate = FrozenBuffer::sampleRate(media);
    const size_t wanted = size_t(std::ceil(capacity * double(sourceRate) / targetRate)) + 64;
    const size_t count = size_t(std::min<drwav_uint64>(wav.totalPCMFrameCount, wanted));
    std::vector<float> raw(count * channels);
    const auto read = drwav_read_pcm_frames_f32(&wav, count, raw.data());
    drwav_uninit(&wav);
    if (read != count) return false;
    // Resample outside the callback, with finite, bounded samples.
    std::vector<float> stereo((count + 64) * 2, 0.f);
    for (size_t i = 0; i < count; ++i) {
        for (unsigned ch = 0; ch < 2; ++ch) {
            const float value = raw[i * channels + (channels == 1 ? 0 : ch)];
            stereo[2 * i + ch] = std::isfinite(value) ? std::clamp(value, -1.f, .999999f) : 0.f;
        }
    }
    const int frames = std::min(capacity, int(std::ceil(count * double(targetRate) / sourceRate)));
    std::vector<float> converted(size_t(frames) * 2);
    if (sourceRate == unsigned(targetRate)) {
        std::copy_n(stereo.data(), converted.size(), converted.data());
    } else {
        int error = 0;
        auto* resampler = speex_resampler_init(2, sourceRate, targetRate, 5, &error);
        if (!resampler || error) { if (resampler) speex_resampler_destroy(resampler); return false; }
        speex_resampler_skip_zeros(resampler);
        const size_t padding = size_t(speex_resampler_get_input_latency(resampler)) + 64;
        stereo.resize((count + padding) * 2, 0.f);
        spx_uint32_t consumed = count + padding, produced = frames;
        error = speex_resampler_process_interleaved_float(resampler, stereo.data(), &consumed,
                                                         converted.data(), &produced);
        speex_resampler_destroy(resampler);
        if (error || produced != unsigned(frames)) return false;
    }
    std::vector<beads::FloatFrame> output(frames);
    for (int i = 0; i < frames; ++i) output[i] = {converted[2 * i], converted[2 * i + 1]};
    return BeadsEngine::importFrozenBuffer(output.data(), frames, media, channels == 1, buffer);
}

} // namespace audible_beads
