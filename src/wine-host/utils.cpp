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

#include <algorithm>
#include <cctype>
#include <codecvt>
#include <iostream>
#include <locale>
#include <optional>
#include <system_error>

#include "bridges/common.h"

using namespace std::literals::chrono_literals;

namespace {

// Empirically `wine_get_dos_file_name()` only canonicalizes a file's path
// to its drive-rooted DOS form for a small set of recognized image
// extensions (`.dll`, `.exe`, `.sys`, `.drv`). For every other extension
// (including the ones we care about: `.vst3`, `.clap`, plus the generic
// VST2 `.dll` case that does work) Wine returns a raw NT-namespace path
// like `\\?\unix\home\...` even when the file lives under a registered
// drive. We treat that as a non-result and fall through to a manual
// dosdevices walk.
std::optional<std::string> wine_convert_to_dos(const std::string& unix_path) {
    WCHAR* converted = wine_get_dos_file_name(unix_path.c_str());
    if (!converted) return std::nullopt;

    static_assert(sizeof(WCHAR) == sizeof(char16_t));
    // wstring_convert::to_bytes throws std::range_error on conversion
    // failure (malformed UTF-16 input).  Caller (to_dos_path) is
    // declared noexcept and would terminate on escape.  Catch here and
    // return nullopt so the caller naturally falls through to
    // manual_dosdevices_lookup.  Same noexcept-violation bug class as
    // editor.cpp::redetect_host_window fixed in commit 1f979647 — see
    // ~/wine-nspa-notes/yabridge-noexcept-audit-20260513.md.
    //
    // Always HeapFree the Wine-allocated buffer before returning,
    // regardless of conversion outcome.
    std::optional<std::string> result;
    try {
        std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t>
            converter;
        result = converter.to_bytes(
            std::u16string(reinterpret_cast<char16_t*>(converted)));
    } catch (const std::range_error&) {
        // Malformed UTF-16 from Wine — extremely unlikely for a real
        // file path, but defensively return nullopt rather than letting
        // the exception escape the noexcept boundary at to_dos_path.
    }
    HeapFree(GetProcessHeap(), 0, converted);

    if (!result || result->compare(0, 8, "\\\\?\\unix") == 0) {
        return std::nullopt;
    }
    return result;
}

// Walk `$WINEPREFIX/dosdevices/` and find the drive whose symlink target
// is the longest unix prefix of `unix_path`. Returns the DOS path on a
// successful match, std::nullopt otherwise. This mirrors what Wine itself
// would do if it routed all extensions through its drive-letter mapping.
std::optional<std::string> manual_dosdevices_lookup(
    const std::string& unix_path) {
    const char* prefix_env = std::getenv("WINEPREFIX");
    if (!prefix_env) return std::nullopt;

    namespace fs = ghc::filesystem;
    std::error_code ec;
    fs::path dosdevices = fs::path(prefix_env) / "dosdevices";
    if (!fs::is_directory(dosdevices, ec)) return std::nullopt;

    std::string best_letter;
    size_t best_match_len = 0;

    for (auto it = fs::directory_iterator(dosdevices, ec);
         !ec && it != fs::directory_iterator(); it.increment(ec)) {
        const std::string name = it->path().filename().string();
        // Drive letter entries are exactly `X:` — skip `X::` block-device
        // aliases (those point at /dev/* nodes, not directories that
        // would plausibly contain a plugin).
        if (name.length() != 2 || name[1] != ':' ||
            !std::isalpha(static_cast<unsigned char>(name[0]))) {
            continue;
        }

        std::error_code resolve_ec;
        fs::path target = fs::canonical(it->path(), resolve_ec);
        if (resolve_ec) continue;
        std::string target_str = target.string();
        if (!target_str.empty() && target_str.back() != '/') target_str += '/';

        if (unix_path.length() >= target_str.length() &&
            unix_path.compare(0, target_str.length(), target_str) == 0 &&
            target_str.length() > best_match_len) {
            best_letter = std::string(
                1, static_cast<char>(
                       std::toupper(static_cast<unsigned char>(name[0]))));
            best_match_len = target_str.length();
        }
    }

    if (best_match_len == 0) return std::nullopt;

    std::string dos = best_letter + ":\\" + unix_path.substr(best_match_len);
    std::replace(dos.begin(), dos.end(), '/', '\\');
    return dos;
}

}  // namespace

std::string to_dos_path(const std::string& unix_path) noexcept {
    if (auto r = wine_convert_to_dos(unix_path)) return *r;
    if (auto r = manual_dosdevices_lookup(unix_path)) return *r;
    return unix_path;
}

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
