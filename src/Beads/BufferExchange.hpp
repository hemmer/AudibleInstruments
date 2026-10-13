#pragma once
#include "BeadsEngine.hpp"
#include <atomic>

namespace audible_beads {

// One publisher, multiple readers. Audio never waits, allocates, or releases
// heap storage. Readers pin a slot while files are written or state is applied.
class BufferExchange {
    struct Slot {
        FrozenBuffer buffer;
        std::atomic<unsigned> readers{0};
        uint64_t revision = 0;
    };
    std::unique_ptr<std::array<Slot, 3>> slots_{new std::array<Slot, 3>};
    std::atomic<int> published_{-1};
    uint64_t nextRevision_ = 0;
    std::atomic<uint64_t> announcedRevision_{0};
    int staging_ = -1; // Publisher-only, never visible until complete.
public:
    void clear() { published_.store(-1); announcedRevision_.store(0); }
    uint64_t revision() const { return announcedRevision_.load(); }
    void cancelWrite() { staging_ = -1; }
    template<class Writer> bool publishStep(Writer writer) {
        if (staging_ < 0) {
            const int current = published_.load();
            for (int i = 0; i < 3; ++i) {
                if (i != current && !(*slots_)[i].readers.load()) { staging_ = i; break; }
            }
        }
        if (staging_ < 0) return false;
        auto& slot = (*slots_)[staging_];
        if (!writer(slot.buffer)) return false;
        slot.revision = ++nextRevision_;
        published_.store(staging_);
        announcedRevision_.store(slot.revision);
        staging_ = -1;
        return true;
    }
    template<class Writer> bool publish(Writer writer) {
        const int current = published_.load();
        for (int i = 0; i < 3; ++i) {
            auto& slot = (*slots_)[i];
            if (i == current || slot.readers.load()) continue;
            if (!writer(slot.buffer)) return false;
            slot.revision = ++nextRevision_;
            published_.store(i);
            announcedRevision_.store(slot.revision);
            return true;
        }
        return false;
    }
    template<class Reader> bool read(Reader reader) const {
        for (int attempt = 0; attempt < 8; ++attempt) {
            const int index = published_.load();
            if (index < 0) return false;
            auto& slot = (*slots_)[index];
            slot.readers.fetch_add(1);
            struct Pin {
                std::atomic<unsigned>& readers;
                ~Pin() { readers.fetch_sub(1); }
            } pin{slot.readers};
            if (published_.load() != index) {
                continue;
            }
            reader(slot.buffer, slot.revision);
            return true;
        }
        return false;
    }
};

} // namespace audible_beads
