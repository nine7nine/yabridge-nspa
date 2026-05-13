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

#include "audio-control-shm.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace yabridge::nspa {

// Direct-struct envelope rollout gate.  Returns the cached result of
// reading YABRIDGE_DIRECT_ENVELOPE at process startup.  See header for
// rollout policy.  Subsequent calls return without touching the
// environment, so the function is RT-safe after first invocation —
// call sites should trigger the initial getenv before the audio thread
// starts (typical pattern: read once during AudioControlShm construction).
bool direct_envelope_enabled() noexcept {
    static const bool enabled = [] {
        // NOLINTNEXTLINE(concurrency-mt-unsafe)
        const char* val = std::getenv(direct_envelope_env_var);
        return val != nullptr && val[0] == '1' && val[1] == '\0';
    }();
    return enabled;
}

namespace {
// Helper to destroy a partially-constructed Layout on init failure.
// Idempotent — checks current state via the futex word; doesn't
// re-destroy already-destroyed primitives.
void cleanup_partial_init(AudioControlShmLayout* layout,
                          bool req_lock_inited,
                          bool req_cv_inited,
                          bool reply_lock_inited,
                          bool reply_cv_inited) noexcept {
    if (reply_cv_inited)   pi_cond_destroy(&layout->reply_cv);
    if (reply_lock_inited) pi_mutex_destroy(&layout->reply_lock);
    if (req_cv_inited)     pi_cond_destroy(&layout->req_cv);
    if (req_lock_inited)   pi_mutex_destroy(&layout->req_lock);
}
}  // namespace

AudioControlShm::AudioControlShm(Create, const std::string& name)
    : name_(name), is_creator_(true) {
    // O_EXCL — fail if a stale region from a crashed predecessor exists.
    // The caller is expected to clean those up (typically via a higher-
    // level handshake teardown). Permissive (0600) for our user only.
    fd_ = shm_open(name_.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
    if (fd_ < 0) {
        throw std::system_error(errno, std::system_category(),
                                "shm_open(create) " + name_);
    }

    if (ftruncate(fd_, sizeof(AudioControlShmLayout)) != 0) {
        const int err = errno;
        close(fd_);
        shm_unlink(name_.c_str());
        throw std::system_error(err, std::system_category(),
                                "ftruncate " + name_);
    }

    // MAP_LOCKED — the audio rendezvous must never be paged out. Falls
    // back to non-locked on permission failure.
    void* p = mmap(nullptr, sizeof(AudioControlShmLayout),
                   PROT_READ | PROT_WRITE,
                   MAP_SHARED | MAP_LOCKED, fd_, 0);
    if (p == MAP_FAILED) {
        p = mmap(nullptr, sizeof(AudioControlShmLayout),
                 PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (p == MAP_FAILED) {
            const int err = errno;
            close(fd_);
            shm_unlink(name_.c_str());
            throw std::system_error(err, std::system_category(),
                                    "mmap " + name_);
        }
    }
    layout_ = static_cast<AudioControlShmLayout*>(p);

    // Initialize both pi_mutex+pi_cond pairs PSHARED. Track partial-
    // init state so we can clean up on later failures.
    bool req_lock_ok = false, req_cv_ok = false;
    bool reply_lock_ok = false, reply_cv_ok = false;

    auto on_init_failure = [&](const char* what) {
        const int err = errno;
        cleanup_partial_init(layout_, req_lock_ok, req_cv_ok,
                             reply_lock_ok, reply_cv_ok);
        munmap(layout_, sizeof(AudioControlShmLayout));
        close(fd_);
        shm_unlink(name_.c_str());
        throw std::system_error(err, std::system_category(),
                                std::string(what) + " " + name_);
    };

    if (pi_mutex_init(&layout_->req_lock, RTPI_MUTEX_PSHARED) != 0) {
        on_init_failure("pi_mutex_init(req_lock)");
    }
    req_lock_ok = true;

    if (pi_cond_init(&layout_->req_cv, RTPI_COND_PSHARED) != 0) {
        on_init_failure("pi_cond_init(req_cv)");
    }
    req_cv_ok = true;

    if (pi_mutex_init(&layout_->reply_lock, RTPI_MUTEX_PSHARED) != 0) {
        on_init_failure("pi_mutex_init(reply_lock)");
    }
    reply_lock_ok = true;

    if (pi_cond_init(&layout_->reply_cv, RTPI_COND_PSHARED) != 0) {
        on_init_failure("pi_cond_init(reply_cv)");
    }
    reply_cv_ok = true;

    layout_->state.store(static_cast<uint32_t>(AudioControlState::Idle),
                         std::memory_order_release);
    layout_->request_size = 0;
    layout_->reply_size = 0;
    layout_->generation = 1;

    // === NSPA L2 direct-struct envelope extension init ===
    //
    // Creator writes both layout version and use-direct flag.  Peer
    // reads these on Attach and computes its own local envelope_active_
    // (with version-match check).
    //
    // envelope_use_direct = 1 iff YABRIDGE_DIRECT_ENVELOPE=1 in this
    // process's env.  Direct path is opt-in; default off.
    //
    // Per-format envelope sections are zeroed.  Wine PE-side peer
    // expects a zeroed envelope on first attach; subsequent producer
    // writes happen under the request pi_mutex.
    envelope_active_ = direct_envelope_enabled();
    layout_->envelope_layout_version.store(audio_control_layout_version,
                                           std::memory_order_release);
    layout_->envelope_use_direct = envelope_active_ ? 1u : 0u;
    std::memset(&layout_->request_envelope_vst3, 0,
                sizeof(layout_->request_envelope_vst3));
    std::memset(&layout_->request_envelope_vst2, 0,
                sizeof(layout_->request_envelope_vst2));
    std::memset(&layout_->request_envelope_clap, 0,
                sizeof(layout_->request_envelope_clap));
}

AudioControlShm::AudioControlShm(Attach, const std::string& name)
    : name_(name), is_creator_(false) {
    fd_ = shm_open(name_.c_str(), O_RDWR, 0);
    if (fd_ < 0) {
        throw std::system_error(errno, std::system_category(),
                                "shm_open(attach) " + name_);
    }

    void* p = mmap(nullptr, sizeof(AudioControlShmLayout),
                   PROT_READ | PROT_WRITE,
                   MAP_SHARED | MAP_LOCKED, fd_, 0);
    if (p == MAP_FAILED) {
        p = mmap(nullptr, sizeof(AudioControlShmLayout),
                 PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (p == MAP_FAILED) {
            const int err = errno;
            close(fd_);
            throw std::system_error(err, std::system_category(),
                                    "mmap(attach) " + name_);
        }
    }
    layout_ = static_cast<AudioControlShmLayout*>(p);
    // Peer does NOT re-init the pi_mutex / pi_cond pairs — they're
    // already alive (initialized by the creator). Re-initializing
    // would clobber any in-flight waiters.

    // === NSPA L2 direct-struct envelope extension peer check ===
    //
    // Read creator's layout version and use-direct flag.  If version
    // matches our compile-time `audio_control_layout_version` AND
    // creator opted into direct path, peer activates direct path
    // locally.  Otherwise stays off and the bitsery+shmem path runs
    // unchanged — strict, transparent fallback.
    //
    // We read envelope_layout_version with acquire ordering to pair
    // with the creator's release-store at the end of Create.  This
    // ensures the rest of the envelope memory (request_envelope_vst3
    // = zeroed) is visible to us.
    const uint32_t creator_version =
        layout_->envelope_layout_version.load(std::memory_order_acquire);
    const uint32_t creator_use_direct = layout_->envelope_use_direct;
    if (creator_version == audio_control_layout_version &&
        creator_use_direct == 1u) {
        envelope_active_ = true;
    } else {
        envelope_active_ = false;
    }
}

AudioControlShm::~AudioControlShm() noexcept {
    if (is_moved_) return;

    if (layout_) {
        // Only the creator destroys the sync primitives. Doing so from
        // the peer side would corrupt the futex state for a still-
        // active creator.
        if (is_creator_) {
            pi_cond_destroy(&layout_->reply_cv);
            pi_mutex_destroy(&layout_->reply_lock);
            pi_cond_destroy(&layout_->req_cv);
            pi_mutex_destroy(&layout_->req_lock);
        }
        munmap(layout_, sizeof(AudioControlShmLayout));
    }
    if (fd_ >= 0) {
        close(fd_);
    }
    if (is_creator_) {
        shm_unlink(name_.c_str());
    }
}

void AudioControlShm::signal_shutdown() noexcept {
    if (!layout_) return;

    // Set state atomically — both directions check this under their
    // respective lock + on wake from pi_cond_wait.
    layout_->state.store(static_cast<uint32_t>(AudioControlState::Shutdown),
                         std::memory_order_release);

    // Wake any waiter on the request direction. The wine-host audio
    // thread is most likely here (parked in audio_control_recv_one's
    // wait-for-request loop).
    if (pi_mutex_lock(&layout_->req_lock) == 0) {
        pi_cond_broadcast(&layout_->req_cv, &layout_->req_lock);
        pi_mutex_unlock(&layout_->req_lock);
    }

    // Wake any waiter on the reply direction. The plugin-lib audio
    // thread is most likely here (parked in audio_control_send_and_wait's
    // wait-for-reply loop) if a callback was in flight at teardown.
    if (pi_mutex_lock(&layout_->reply_lock) == 0) {
        pi_cond_broadcast(&layout_->reply_cv, &layout_->reply_lock);
        pi_mutex_unlock(&layout_->reply_lock);
    }
}

AudioControlShm::AudioControlShm(AudioControlShm&& o) noexcept
    : name_(std::move(o.name_)),
      fd_(o.fd_),
      layout_(o.layout_),
      is_creator_(o.is_creator_),
      is_moved_(false),
      envelope_active_(o.envelope_active_) {
    o.is_moved_ = true;
    o.fd_ = -1;
    o.layout_ = nullptr;
    o.envelope_active_ = false;
}

// =====================================================================
// Hot-path helpers
// =====================================================================

void audio_control_send_and_wait(AudioControlShm& shm,
                                 const uint8_t* request_bytes,
                                 size_t request_size,
                                 uint8_t* reply_bytes_out,
                                 size_t reply_bytes_capacity,
                                 size_t& reply_size_out) {
    if (request_size > audio_control_buf_size) {
        throw std::overflow_error(
            "audio_control_send_and_wait: request too large for shmem region");
    }

    auto& layout = shm.layout();

    // === Publish request ===
    pi_mutex_t* req_mu = shm.req_lock();
    pi_cond_t*  req_cv = shm.req_cv();

    if (pi_mutex_lock(req_mu) != 0) {
        throw std::system_error(errno, std::system_category(),
                                "pi_mutex_lock(req) in send_and_wait");
    }

    // Check for shutdown before publishing — if we're tearing down,
    // skip the round-trip entirely.
    if (layout.state.load(std::memory_order_acquire) ==
        static_cast<uint32_t>(AudioControlState::Shutdown)) {
        pi_mutex_unlock(req_mu);
        throw AudioControlShutdown{};
    }

    std::memcpy(layout.request_buf, request_bytes, request_size);
    layout.request_size = static_cast<uint32_t>(request_size);
    layout.state.store(static_cast<uint32_t>(AudioControlState::RequestReady),
                       std::memory_order_release);

    // Wake the consumer with PI boost — the kernel applies our
    // effective priority to the popping task via FUTEX_CMP_REQUEUE_PI.
    if (pi_cond_signal(req_cv, req_mu) != 0) {
        pi_mutex_unlock(req_mu);
        throw std::system_error(errno, std::system_category(),
                                "pi_cond_signal(req) in send_and_wait");
    }

    pi_mutex_unlock(req_mu);
    // ===== Request lock released; consumer can now process =====

    // === Wait for reply ===
    pi_mutex_t* reply_mu = shm.reply_lock();
    pi_cond_t*  reply_cv = shm.reply_cv();

    if (pi_mutex_lock(reply_mu) != 0) {
        throw std::system_error(errno, std::system_category(),
                                "pi_mutex_lock(reply) in send_and_wait");
    }

    for (;;) {
        const uint32_t s = layout.state.load(std::memory_order_acquire);
        if (s == static_cast<uint32_t>(AudioControlState::ReplyReady)) {
            break;
        }
        if (s == static_cast<uint32_t>(AudioControlState::Shutdown)) {
            pi_mutex_unlock(reply_mu);
            throw AudioControlShutdown{};
        }
        if (pi_cond_wait(reply_cv, reply_mu) != 0) {
            pi_mutex_unlock(reply_mu);
            throw std::system_error(errno, std::system_category(),
                                    "pi_cond_wait(reply) in send_and_wait");
        }
    }

    const size_t reply_size = layout.reply_size;
    if (reply_size > reply_bytes_capacity) {
        pi_mutex_unlock(reply_mu);
        throw std::overflow_error(
            "audio_control_send_and_wait: reply too large for caller buffer");
    }

    std::memcpy(reply_bytes_out, layout.reply_buf, reply_size);
    reply_size_out = reply_size;
    layout.state.store(static_cast<uint32_t>(AudioControlState::Idle),
                       std::memory_order_release);

    pi_mutex_unlock(reply_mu);
}

}  // namespace yabridge::nspa
