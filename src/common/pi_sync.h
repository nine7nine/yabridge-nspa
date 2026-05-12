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

// C++ RAII wrappers around the vendored librtpi primitives in rtpi.h.
//
// PiMutex and PiCond satisfy the C++ BasicLockable / Lockable concepts
// (lock / unlock / try_lock), so they are drop-in replacements for
// std::mutex in std::lock_guard, std::unique_lock, and std::scoped_lock.
// std::condition_variable is NOT compatible — it is hard-coded to
// std::unique_lock<std::mutex>. PiCond + PiMutex form a parallel
// (cv, mutex) pair using rtpi.h's pi_cond_signal / pi_cond_wait under
// the hood; use std::condition_variable_any with PiMutex if you need
// to mix a different cv with a PiMutex.
//
// Two distinct cross-process modes are supported:
//
//   PiMutex()             — process-private mutex (FUTEX_LOCK_PI_PRIVATE)
//   PiMutex(PiShared{})   — process-shared (FUTEX_LOCK_PI), suitable for
//                           placement-new into a shmem region shared with
//                           another process.
//
// The same applies to PiCond. Cross-process pi_mutex_t works on stock
// PREEMPT_RT kernels via the standard FUTEX_LOCK_PI / FUTEX_UNLOCK_PI
// path — no Wine-NSPA-specific kernel extension required.
//
// Lifetime note for shmem regions: the destructor calls pi_mutex_destroy
// (memset zero). When placement-new'd into shmem, do not invoke the
// destructor from a process that may still have peers using the region.
// Either skip the destructor (the shmem unlink path zeroes the bytes
// anyway) or coordinate teardown.

#include <stdexcept>
#include <system_error>

#include "rtpi.h"

namespace yabridge::nspa {

// Tag type to construct a process-shared pi primitive. Use as
// PiMutex(PiShared{}) / PiCond(PiShared{}).
struct PiShared {};

// RAII wrapper around pi_mutex_t. BasicLockable + Lockable.
//
// Non-copyable, non-movable: the pi_mutex_t stores a futex word whose
// address must remain stable for the life of the lock (kernel waiter
// queues are keyed by physical address for PSHARED, virtual address
// otherwise).
class PiMutex {
   public:
    PiMutex() noexcept {
        // Return value can only be EINVAL from invalid flags; flags=0 is
        // always valid, so this cannot fail.
        pi_mutex_init(&m_, 0);
    }

    explicit PiMutex(PiShared /*tag*/) noexcept {
        pi_mutex_init(&m_, RTPI_MUTEX_PSHARED);
    }

    ~PiMutex() noexcept { pi_mutex_destroy(&m_); }

    PiMutex(const PiMutex&) = delete;
    PiMutex& operator=(const PiMutex&) = delete;
    PiMutex(PiMutex&&) = delete;
    PiMutex& operator=(PiMutex&&) = delete;

    void lock() {
        const int err = pi_mutex_lock(&m_);
        if (err != 0) {
            throw std::system_error(err, std::system_category(),
                                    "pi_mutex_lock");
        }
    }

    void unlock() noexcept {
        // pi_mutex_unlock returns EPERM if the caller is not the owner;
        // that's a programming error in this RAII context and unlock is
        // noexcept by convention. We swallow the return value rather than
        // throwing from a destructor-time unlock (std::lock_guard).
        pi_mutex_unlock(&m_);
    }

    [[nodiscard]] bool try_lock() noexcept {
        return pi_mutex_trylock(&m_) == 0;
    }

    // Direct access to the underlying pi_mutex_t for PiCond, which needs
    // to pass it to pi_cond_wait / pi_cond_signal.
    pi_mutex_t* native_handle() noexcept { return &m_; }

   private:
    pi_mutex_t m_;
};

// RAII wrapper around pi_cond_t. Always paired with a PiMutex.
//
// Same non-copyable / non-movable rationale as PiMutex.
class PiCond {
   public:
    PiCond() noexcept { pi_cond_init(&c_, 0); }

    explicit PiCond(PiShared /*tag*/) noexcept {
        pi_cond_init(&c_, RTPI_COND_PSHARED);
    }

    ~PiCond() noexcept { pi_cond_destroy(&c_); }

    PiCond(const PiCond&) = delete;
    PiCond& operator=(const PiCond&) = delete;
    PiCond(PiCond&&) = delete;
    PiCond& operator=(PiCond&&) = delete;

    // Caller must hold `mutex` on entry. Releases it for the duration of
    // the wait and reacquires it before returning. As with any condvar,
    // wrap in a `while (!predicate) cond.wait(mutex);` loop — spurious
    // wakes and EAGAIN retries can occur.
    void wait(PiMutex& mutex) {
        const int err = pi_cond_wait(&c_, mutex.native_handle());
        if (err != 0) {
            throw std::system_error(err, std::system_category(),
                                    "pi_cond_wait");
        }
    }

    // Wake exactly one waiter; the woken thread is requeued onto the
    // mutex's PI chain so no inversion gap exists between wake and lock.
    void signal(PiMutex& mutex) noexcept {
        pi_cond_signal(&c_, mutex.native_handle());
    }

    void broadcast(PiMutex& mutex) noexcept {
        pi_cond_broadcast(&c_, mutex.native_handle());
    }

   private:
    pi_cond_t c_;
};

// Placement-new helpers for shmem regions. The shmem creator calls
// PiMutex::init_shared / PiCond::init_shared on the raw shmem memory;
// peer processes that mmap the same region get a fully usable
// PiMutex&/PiCond& by reinterpret_cast over the same bytes.
//
// We don't use the C++ constructors directly on shmem memory because
// the constructor runs pi_mutex_init / pi_cond_init, and those
// operations are not idempotent across re-attaching processes — only
// the creator should initialize.

inline void pi_mutex_init_shared(pi_mutex_t* m) noexcept {
    pi_mutex_init(m, RTPI_MUTEX_PSHARED);
}

inline void pi_cond_init_shared(pi_cond_t* c) noexcept {
    pi_cond_init(c, RTPI_COND_PSHARED);
}

}  // namespace yabridge::nspa
