// yabridge: a Wine plugin bridge
// Copyright (C) 2020-2026 Robbert van der Helm
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

#include "utils.h"

// NSPA: pidfd_open syscall for the event-driven watchdog. Kernel ≥ 5.3
// for pidfd_open itself; PIDFD_NONBLOCK requires kernel ≥ 5.10. We
// fall back to PIDFD_NONBLOCK=0 if the header doesn't define it (very
// old glibc); pidfd_open will return a blocking fd, which is still
// fine because async_wait only POLLINs, never reads bytes.
#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>
#ifndef PIDFD_NONBLOCK
#define PIDFD_NONBLOCK O_NONBLOCK
#endif
#ifndef SYS_pidfd_open
#define SYS_pidfd_open 434  // x86_64
#endif

#include <iostream>

#include "bridges/common.h"

using namespace std::literals::chrono_literals;

uint32_t WINAPI
win32_thread_trampoline(fu2::unique_function<void()>* entry_point) {
    (*entry_point)();
    delete entry_point;

    return 0;
}

Win32Thread::Win32Thread() noexcept : handle_(nullptr, nullptr) {}

Win32Thread::~Win32Thread() noexcept {
    if (handle_) {
        WaitForSingleObject(handle_.get(), INFINITE);
    }
}

Win32Thread::Win32Thread(Win32Thread&& o) noexcept
    : handle_(std::move(o.handle_)) {
    o.handle_.reset();
}

Win32Thread& Win32Thread::operator=(Win32Thread&& o) noexcept {
    handle_ = std::move(o.handle_);
    o.handle_.reset();

    return *this;
}

Win32Timer::Win32Timer() noexcept {}

Win32Timer::Win32Timer(HWND window_handle,
                       size_t timer_id,
                       unsigned int interval_ms) noexcept
    : window_handle_(window_handle), timer_id_(timer_id) {
    SetTimer(window_handle, timer_id, interval_ms, nullptr);
}

Win32Timer::~Win32Timer() noexcept {
    if (timer_id_) {
        KillTimer(window_handle_, *timer_id_);
    }
}

Win32Timer::Win32Timer(Win32Timer&& o) noexcept
    : window_handle_(o.window_handle_), timer_id_(std::move(o.timer_id_)) {
    o.timer_id_.reset();
}

Win32Timer& Win32Timer::operator=(Win32Timer&& o) noexcept {
    window_handle_ = o.window_handle_;
    timer_id_ = std::move(o.timer_id_);
    o.timer_id_.reset();

    return *this;
}

MainContext::MainContext()
    : context_(),
      events_timer_(context_),
      watchdog_context_(),
      watchdog_timer_(watchdog_context_) {}

void MainContext::run() {
    // We need to know which thread is the GUI thread because mutual recursion
    // in VST3 plugins needs to be handled differently depending on whether the
    // potentially mutually recursive function was called from an audio thread
    // or a GUI thread
    gui_thread_id_ = GetCurrentThreadId();

    // NOTE: We allow disabling the watchdog timer to allow the Wine process to
    //       be run from a separate namespace. This is not something you'd
    //       normally want to enable.
    if (is_watchdog_timer_disabled()) {
        std::cerr << "WARNING: Watchdog timer disabled. Not protecting"
                  << std::endl;
        std::cerr << "         against dangling processes." << std::endl;
    } else {
        // To account for hosts terminating before the bridged plugin has
        // initialized, we'll do the first watchdog check five seconds. After
        // this we'll run the timer on a 30 second interval.
        async_handle_watchdog_timer(5s);

        watchdog_handler_ = Win32Thread([&]() {
            pthread_setname_np(pthread_self(), "watchdog");

            watchdog_context_.run();
        });
    }

    context_.run();

    // We only need to check if the host is still running while the main context
    // is also running. If a stop was requested, the entire application is
    // supposed to shut down. Otherwise `watchdog_handler` would just block on
    // the join as the watchdog timer is still active.
    watchdog_context_.stop();
}

void MainContext::stop() noexcept {
    context_.stop();
}

void MainContext::update_timer_interval(
    std::chrono::steady_clock::duration new_interval) noexcept {
    timer_interval_ = new_interval;
}

MainContext::WatchdogGuard::WatchdogGuard(HostBridge& bridge,
                                          MainContext& main_context,
                                          pid_t parent_pid)
    : bridge_(&bridge), main_context_(main_context) {
    std::lock_guard lock(main_context.watched_bridges_mutex_);
    main_context.watched_bridges_.insert(&bridge);

    // NSPA: open a pidfd on the parent and async_wait POLLIN via the
    // watchdog io_context. The async handler fires shutdown_if_dangling
    // the instant the kernel sees the parent exit — zero polling
    // latency. The handler takes the same mutex and checks the bridge
    // is still in watched_bridges_ before dereferencing, so a teardown
    // race (bridge destruction concurrent with handler scheduling)
    // can't UAF.
    //
    // syscall(SYS_pidfd_open, ...) — pidfd_open(2), kernel ≥ 5.3.
    // CLOEXEC by default; we set O_NONBLOCK so async_wait doesn't ever
    // need to block on read (we only POLLIN, never read bytes).
    const int pidfd =
        static_cast<int>(syscall(SYS_pidfd_open, parent_pid, PIDFD_NONBLOCK));
    if (pidfd >= 0) {
        try {
            auto [it, inserted] = main_context.pidfd_watches_.try_emplace(
                &bridge, main_context.watchdog_context_, pidfd);
            if (inserted) {
                MainContext* mc = &main_context;
                HostBridge* bp = &bridge;
                it->second.async_wait(
                    asio::posix::stream_descriptor::wait_read,
                    [mc, bp](const std::error_code& ec) {
                        if (ec) {
                            // operation_aborted from teardown, or some
                            // transient — leave the 30s timer to catch up.
                            return;
                        }
                        std::lock_guard lock(mc->watched_bridges_mutex_);
                        if (mc->watched_bridges_.contains(bp)) {
                            bp->shutdown_if_dangling();
                        }
                    });
            } else {
                // Already had a watch for this bridge — shouldn't happen.
                ::close(pidfd);
            }
        } catch (const std::exception&) {
            ::close(pidfd);
        }
    }
    // On pidfd_open failure (older kernel, ENOMEM, etc.) the 30s polling
    // watchdog still catches the dangling case — just with up to 30s
    // detection latency instead of instant.
}

MainContext::WatchdogGuard::~WatchdogGuard() noexcept {
    if (is_active_) {
        main_context_.get().unregister_watchdog(*bridge_);
    }
}

MainContext::WatchdogGuard::WatchdogGuard(WatchdogGuard&& o) noexcept
    : bridge_(std::move(o.bridge_)),
      main_context_(std::move(o.main_context_)) {
    o.is_active_ = false;
}

MainContext::WatchdogGuard& MainContext::WatchdogGuard::operator=(
    WatchdogGuard&& o) noexcept {
    bridge_ = std::move(o.bridge_);
    main_context_ = std::move(o.main_context_);
    o.is_active_ = false;

    return *this;
}

MainContext::WatchdogGuard MainContext::register_watchdog(HostBridge& bridge,
                                                          pid_t parent_pid) {
    // The guard's constructor and destructor will handle actually registering
    // and unregistering the bridge from `watched_bridges_` and
    // `pidfd_watches_`.
    return WatchdogGuard(bridge, *this, parent_pid);
}

void MainContext::unregister_watchdog(HostBridge& bridge) noexcept {
    std::lock_guard lock(watched_bridges_mutex_);
    watched_bridges_.erase(&bridge);
    // Erasing the pidfd_watches_ entry destroys the stream_descriptor,
    // which cancels any pending async_wait (the handler fires once
    // with operation_aborted; our handler early-returns on that) and
    // closes the underlying pidfd.
    pidfd_watches_.erase(&bridge);
}

void MainContext::async_handle_watchdog_timer(
    std::chrono::steady_clock::duration interval) {
    watchdog_timer_.expires_at(std::chrono::steady_clock::now() + interval);
    watchdog_timer_.async_wait([&](const std::error_code& error) {
        if (error) {
            return;
        }

        // When the `WatchdogGuard` field on `HostBridge` gets destroyed, that
        // bridge instance will be removed from `watched_bridges`. So if our
        // call to `HostBridge::shutdown_if_dangling()` shuts the plugin down,
        // the instance will be removed after this lambda exits.
        std::lock_guard lock(watched_bridges_mutex_);
        for (auto& bridge : watched_bridges_) {
            bridge->shutdown_if_dangling();
        }

        async_handle_watchdog_timer(30s);
    });
}
