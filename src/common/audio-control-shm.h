// yabridge — Wine-NSPA integration
// Copyright (C) 2026 jordan Johnston <johnstonljordan@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

// AudioControlShm — process-shared rendezvous region for one audio
// request/reply round-trip, using two pi_mutex + pi_cond pairs (one per
// direction) from the vendored rtpi.h.
//
// This is the L2 transport for the audio-callback hot path. Replaces the
// unix-socket round-trip in libyabridge<->yabridge-host with a userspace
// pi_cond wait. Two semantic wins:
//
//   1. Cross-process priority inheritance. The plugin-lib side (in the
//      DAW process, on the DAW's audio thread) signals the wine-host
//      side; the kernel requeues the waiter onto the PI mutex with PI
//      boost from the signaller. Wine-host worker thread runs at the
//      DAW's effective priority for the duration of the callback, then
//      drops back. No need for yabridge's old 10s polling sched_priority
//      sync — the kernel does it per-callback.
//
//   2. Lower per-callback latency. Stock socket round-trip is sendmsg +
//      kernel buffer + recvmsg + scheduler hops on both ends. Futex
//      round-trip is one wake/wait pair plus one CAS. At small audio
//      buffer sizes (64-128 frames) the saved microseconds matter.
//
// Design history (2026-05-11):
//
//   v1 of this region used a SINGLE pi_mutex + pi_cond pair and held the
//   pi_mutex across the entire round-trip including the plugin
//   processReplacing call on the consumer side. That violated the RT
//   contract (cross-process mutex held during arbitrary plugin code) and
//   was reverted (yabridge commit c88e92b4 on nspa-integration branch).
//
//   v2 (this file) uses two pairs — one for request direction, one for
//   reply direction — and the consumer RELEASES the request pi_mutex
//   BEFORE invoking the plugin handler. Plugin processing happens with
//   no cross-process lock held. The consumer reacquires the reply lock
//   only to publish the reply. Pi_mutex hold spans are µs-bounded.
//
// Architecture (v2):
//
//   Producer (libyabridge plugin-lib side, on the DAW's audio thread):
//
//     pi_mutex_lock(&req_lock)
//     memcpy(request_buf, serialized_request)
//     request_size = N
//     state = REQUEST_READY
//     pi_cond_signal(&req_cv)            // wakes consumer with PI boost
//     pi_mutex_unlock(&req_lock)         // request lock released
//
//     pi_mutex_lock(&reply_lock)
//     while (state != REPLY_READY)
//         pi_cond_wait(&reply_cv)        // sleeps until reply arrives
//     read reply_buf, reply_size
//     state = IDLE
//     pi_mutex_unlock(&reply_lock)
//
//   Consumer (yabridge-host wine-host side, on the audio worker thread):
//
//     pi_mutex_lock(&req_lock)
//     while (state != REQUEST_READY)
//         pi_cond_wait(&req_cv)          // boosted by signaller on wake
//     read request_buf, request_size into local buffer
//     pi_mutex_unlock(&req_lock)         // request lock released
//
//     <process audio callback in plugin DLL — NO cross-process lock>
//
//     pi_mutex_lock(&reply_lock)
//     memcpy(reply_buf, serialized_reply)
//     reply_size = N
//     state = REPLY_READY
//     pi_cond_signal(&reply_cv)          // wakes producer
//     pi_mutex_unlock(&reply_lock)
//
// Shutdown:
//
//   Both producer and consumer check the state atomically under their
//   respective lock. signal_shutdown() flips state to SHUTDOWN and
//   broadcasts both cond vars. Whichever direction the audio thread is
//   currently waiting in, it wakes, observes SHUTDOWN, and throws
//   AudioControlShutdown for the loop to catch.
//
// Lifecycle:
//
//   - Plugin-lib is the CREATOR. AudioControlShm(Create{}, name) does
//     shm_open(O_CREAT|O_EXCL) + ftruncate + mmap(MAP_LOCKED) + init
//     both pi_mutex+pi_cond pairs PSHARED + state = Idle.
//
//   - Wine-host is the PEER. AudioControlShm(Attach{}, name) does
//     shm_open(O_RDWR) + mmap. Does NOT re-init the sync primitives.
//
//   - Creator destructor: pi_*_destroy + munmap + close + shm_unlink.
//   - Peer destructor:    munmap + close (no shm_unlink — creator owns).
//
//   Member declaration order in bridge classes is critical: the
//   AudioControlShm member must be declared so it destructs AFTER any
//   Win32Thread member that uses it. Reverse-of-declaration destruction
//   order applies. The Vst2Bridge fix declares AudioControlShm FIRST.
//
// Crash robustness: pi_mutex with NSPA's rtpi.h doesn't support
// RTPI_MUTEX_ROBUST. If the DAW crashes mid-callback the wine-host
// worker stays blocked. A separate pidfd watchdog (future commit) tears
// the bridge down on DAW death; same defence layer the socket transport
// uses for ECONNRESET.
//
// RT contract:
//
//   - No heap allocation on the audio path. Buffers preallocated at
//     construction (inline storage in SerializationBuffer<64K> members
//     of the bridge classes).
//   - Cross-process mutex hold span is µs-bounded (state transitions
//     and memcpy only — NOT spanning plugin processing).
//   - pi_cond_wait yields (kernel futex syscall, not spin).
//   - PI boost via FUTEX_CMP_REQUEUE_PI on signal — the signaller's
//     effective priority is applied to the woken waiter for the
//     critical section duration.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "rtpi.h"

namespace yabridge::nspa {

// State machine values for the rendezvous slot. Stored as a uint32_t
// atomic in the header. Both directions atomically read this; writes
// happen under the appropriate direction's lock + cond signal.
enum class AudioControlState : uint32_t {
    Idle          = 0,  // No request in flight. Slot is free.
    RequestReady  = 1,  // Producer has written a request; consumer should process.
    ReplyReady    = 2,  // Consumer has written a reply; producer should consume.
    Shutdown      = 3,  // Bridge teardown. Both sides should exit their loops.
};

// Sentinel exception thrown by audio_control_recv_one when it observes
// the Shutdown state. Caught at the audio loop level to break out.
struct AudioControlShutdown : public std::exception {
    const char* what() const noexcept override {
        return "AudioControlShm: shutdown requested";
    }
};

// Maximum bytes a single serialized request or reply can carry through
// the shmem region. Sized to comfortably fit any of the audio request
// structs after bitsery serialization. If a payload overflows this,
// the call site falls back to socket transport for that callback.
constexpr size_t audio_control_buf_size = size_t{64} * 1024;

// Header layout — sits at offset 0 of the shmem region. Two cache-line-
// aligned synchronization PAIRS (one per direction), then state +
// metadata on a separate line, then the fixed-size request/reply
// buffers.
//
// Cache-line alignment for pi_mutex/pi_cond is critical: FUTEX_LOCK_PI's
// kernel waiter hash table buckets by address modulo cache line.
// Crossing a cache line can confuse the kernel's PI chain tracking
// under contention.
struct AudioControlShmLayout {
    // Request direction — producer takes this to publish a request,
    // consumer takes this to receive.
    alignas(64) pi_mutex_t req_lock;
    alignas(64) pi_cond_t  req_cv;

    // Reply direction — consumer takes this to publish a reply,
    // producer takes this to receive.
    alignas(64) pi_mutex_t reply_lock;
    alignas(64) pi_cond_t  reply_cv;

    // State + metadata. State is atomically loaded under either lock.
    // Writes to state happen under the lock for the direction that's
    // publishing the new state value.
    alignas(64) std::atomic<uint32_t> state;
    uint32_t request_size;
    uint32_t reply_size;
    uint32_t generation;
    uint8_t  _state_pad[64 - 16];

    alignas(64) uint8_t request_buf[audio_control_buf_size];
    alignas(64) uint8_t reply_buf[audio_control_buf_size];
};

static_assert(offsetof(AudioControlShmLayout, req_lock) == 0,
              "req_lock must be at offset 0 for ABI stability");
static_assert(sizeof(AudioControlShmLayout) >=
                  (2 * audio_control_buf_size + 320),
              "layout sanity check");

// RAII handle to an AudioControlShm region. Holds the shm_open fd and
// the mmap pointer. Constructor mode controls whether we create
// (creator) or attach (peer).
class AudioControlShm {
   public:
    struct Create {};
    struct Attach {};

    // Creator constructor — plugin-lib side. Allocates a fresh shmem
    // region, sizes it, mmaps it, initializes both pi_mutex+pi_cond
    // pairs with PSHARED flags. Throws on any failure.
    //
    // `name` must be a leading-slash path suitable for shm_open
    // (e.g. "/yabridge-audio-ctl-<plugin>-<random>").
    AudioControlShm(Create, const std::string& name);

    // Peer constructor — wine-host side. Opens an already-created
    // shmem region by name, mmaps it. Does NOT re-initialize the
    // sync primitives. Throws on any failure.
    AudioControlShm(Attach, const std::string& name);

    ~AudioControlShm() noexcept;

    AudioControlShm(const AudioControlShm&) = delete;
    AudioControlShm& operator=(const AudioControlShm&) = delete;
    AudioControlShm(AudioControlShm&& o) noexcept;
    AudioControlShm& operator=(AudioControlShm&&) = delete;

    AudioControlShmLayout& layout() noexcept { return *layout_; }
    const AudioControlShmLayout& layout() const noexcept { return *layout_; }

    pi_mutex_t* req_lock()   noexcept { return &layout_->req_lock; }
    pi_cond_t*  req_cv()     noexcept { return &layout_->req_cv; }
    pi_mutex_t* reply_lock() noexcept { return &layout_->reply_lock; }
    pi_cond_t*  reply_cv()   noexcept { return &layout_->reply_cv; }

    const std::string& name() const noexcept { return name_; }
    bool is_creator() const noexcept { return is_creator_; }

    // Signal both sides to exit their loops. Atomically sets
    // state = Shutdown and broadcasts both conds so any waiter
    // (typically the wine-host audio thread blocked in pi_cond_wait
    // on either direction) wakes and observes the state. Safe to
    // call from either side; idempotent.
    //
    // Must be called BEFORE this object's destructor runs (or at least
    // before the shmem is unmapped by either process). Called from the
    // parent bridge's explicit destructor before its Win32Thread
    // members destruct, so the threads observe the signal and exit
    // before being joined.
    void signal_shutdown() noexcept;

   private:
    std::string name_;
    int fd_ = -1;
    AudioControlShmLayout* layout_ = nullptr;
    bool is_creator_ = false;
    bool is_moved_ = false;
};

// =====================================================================
// Hot-path helpers — used at audio call sites.
//
// `request_bytes` is copied INTO the shmem request slot.
// `reply_bytes_out` receives the reply (caller-provided buffer).
//
// Both functions throw std::overflow_error if payload sizes exceed the
// fixed slot size. Callers fall back to socket transport on overflow.
//
// Other failures (pi_mutex_lock errno, pi_cond_wait errno) throw
// std::system_error.
//
// NB: these helpers do NOT allocate. The buffer pointers must reference
// memory whose lifetime spans the call.
// =====================================================================

// Producer side — DAW's audio thread (plugin-lib).
// Lock-span: only across state transitions + memcpy. Plugin processing
// happens between the two unlock points on the consumer side, so the
// producer's wait on reply_cv is bounded by plugin time only.
void audio_control_send_and_wait(AudioControlShm& shm,
                                 const uint8_t* request_bytes,
                                 size_t request_size,
                                 uint8_t* reply_bytes_out,
                                 size_t reply_bytes_capacity,
                                 size_t& reply_size_out);

// Consumer side — wine-host audio worker thread.
// `handler` is invoked WITHOUT any cross-process lock held. Handler
// receives the request bytes (already copied to caller-provided
// `request_local`) and a writable reply buffer (caller-provided
// `reply_local`). Handler returns the reply byte count.
//
// `Handler` signature:
//   size_t(const uint8_t* request, size_t request_size,
//          uint8_t* reply_out, size_t reply_capacity)
//
// Throws AudioControlShutdown when the Shutdown state is observed.
template <typename Handler>
void audio_control_recv_one(AudioControlShm& shm,
                            uint8_t* request_local,
                            size_t request_local_capacity,
                            uint8_t* reply_local,
                            size_t reply_local_capacity,
                            Handler&& handler) {
    pi_mutex_t* req_mu = shm.req_lock();
    pi_cond_t*  req_cv = shm.req_cv();
    auto& layout = shm.layout();

    // === Request direction ===
    if (pi_mutex_lock(req_mu) != 0) {
        throw std::system_error(errno, std::system_category(),
                                "pi_mutex_lock(req) in recv_one");
    }

    size_t request_size = 0;
    for (;;) {
        const uint32_t s = layout.state.load(std::memory_order_acquire);
        if (s == static_cast<uint32_t>(AudioControlState::RequestReady)) {
            // Snapshot under lock; copy to caller-owned buffer.
            request_size = layout.request_size;
            if (request_size > request_local_capacity) {
                pi_mutex_unlock(req_mu);
                throw std::overflow_error(
                    "audio_control_recv_one: request larger than local buffer");
            }
            std::memcpy(request_local, layout.request_buf, request_size);
            break;
        }
        if (s == static_cast<uint32_t>(AudioControlState::Shutdown)) {
            pi_mutex_unlock(req_mu);
            throw AudioControlShutdown{};
        }
        if (pi_cond_wait(req_cv, req_mu) != 0) {
            pi_mutex_unlock(req_mu);
            throw std::system_error(errno, std::system_category(),
                                    "pi_cond_wait(req) in recv_one");
        }
    }

    pi_mutex_unlock(req_mu);
    // ===== NO cross-process lock held below this point =====

    const size_t reply_size = handler(request_local, request_size,
                                      reply_local, reply_local_capacity);

    if (reply_size > audio_control_buf_size) {
        // Shouldn't happen — handler should respect reply_local_capacity
        // which we passed in. Defence-in-depth: throw rather than
        // corrupt the shmem reply slot.
        throw std::overflow_error(
            "audio_control_recv_one: handler returned oversized reply");
    }

    // === Reply direction ===
    pi_mutex_t* reply_mu = shm.reply_lock();
    pi_cond_t*  reply_cv = shm.reply_cv();
    if (pi_mutex_lock(reply_mu) != 0) {
        throw std::system_error(errno, std::system_category(),
                                "pi_mutex_lock(reply) in recv_one");
    }

    std::memcpy(layout.reply_buf, reply_local, reply_size);
    layout.reply_size = static_cast<uint32_t>(reply_size);
    layout.state.store(static_cast<uint32_t>(AudioControlState::ReplyReady),
                       std::memory_order_release);

    if (pi_cond_signal(reply_cv, reply_mu) != 0) {
        pi_mutex_unlock(reply_mu);
        throw std::system_error(errno, std::system_category(),
                                "pi_cond_signal(reply) in recv_one");
    }

    pi_mutex_unlock(reply_mu);
}

}  // namespace yabridge::nspa
