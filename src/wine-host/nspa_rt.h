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

// Wine-host-side helpers for routing thread RT promotion through the
// Win32 SetThreadPriority API.
//
// Background: yabridge historically called set_realtime_priority(true)
// directly with a default priority of 5 to promote wine-host threads
// to SCHED_FIFO. Under Wine-NSPA, the path that turns Win32 priorities
// into Linux SCHED_FIFO bands is in ntdll's NtSetInformationThread
// (ThreadBasePriority) handler — which is reached by calling
// SetThreadPriority. Bypassing that path means NSPA's prio policy
// (process class awareness, NSPA_RT_PRIO ceiling, TS/RR/FF mode
// honoring, effective-prio boost) is unreachable, and the thread
// lands at a fixed low FIFO band.
//
// The helpers in this file replace direct set_realtime_priority(true)
// calls in wine-host bridge code with SetThreadPriority calls so the
// NSPA mapping takes over. On vanilla Wine, SetThreadPriority's
// existing (partial) mapping still applies — this is strictly an
// improvement, never a regression.
//
// Two patterns:
//
//   set_thread_time_critical()        — set this thread to TIME_CRITICAL
//                                       and leave it there. Use for
//                                       thread-entry promotion where the
//                                       thread lives at RT for its
//                                       entire lifetime (the audio
//                                       worker thread, the dispatch
//                                       event loop).
//
//   ScopedTimeCriticalBoost           — RAII: save current priority,
//                                       set to TIME_CRITICAL, restore
//                                       on scope exit. Use for
//                                       bracketed calls into the plugin
//                                       where spawned audio worker
//                                       threads must inherit RT, but
//                                       the caller's prio should not
//                                       be left elevated.

#include <windows.h>

namespace yabridge::nspa {

// Promote the calling thread to THREAD_PRIORITY_TIME_CRITICAL. Under
// NSPA this triggers NtSetInformationThread(ThreadBasePriority) which
// resolves to SCHED_FIFO at NSPA_RT_PRIO (e.g. 80). Under vanilla Wine
// this routes through Wine's existing (partial) Win32 priority mapping.
//
// noexcept: SetThreadPriority can return FALSE on rare failure paths,
// but we treat this as best-effort — yabridge runs without realtime
// promotion if the syscall fails, which mirrors how the old
// set_realtime_priority(true) path also returned a bool ignored by all
// existing call sites.
inline void set_thread_time_critical() noexcept {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
}

// Restore the calling thread to THREAD_PRIORITY_NORMAL. Used by code
// paths that previously called set_realtime_priority(false), which
// directly demoted to SCHED_OTHER. Routing through SetThreadPriority
// keeps NSPA's process-class-aware nice mapping in effect.
inline void set_thread_normal() noexcept {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
}

// RAII helper: on construction, save the current thread priority and
// promote to TIME_CRITICAL. On destruction, restore the saved value.
//
// This replaces the historical pattern:
//     set_realtime_priority(true);
//     ... plugin call ...
//     set_realtime_priority(false);
// which actively demoted the caller to SCHED_OTHER after the bracketed
// call (clobbering any RT priority the caller previously had). Save +
// restore keeps the caller at whatever it was before the bracket.
//
// Why bracket at all? Some plugin entry points (plugin construction,
// IPluginBase::initialize, certain VST2 dispatch opcodes like
// effSetSampleRate/effSetBlockSize) spawn audio worker threads
// internally. Those threads inherit the creator's scheduling via
// pthread_create's default PTHREAD_INHERIT_SCHED. By boosting the
// caller to TIME_CRITICAL before the plugin call, we ensure spawned
// workers come up at the NSPA-mapped audio band, not at the caller's
// regular priority.
class ScopedTimeCriticalBoost {
   public:
    ScopedTimeCriticalBoost() noexcept
        : saved_priority_(GetThreadPriority(GetCurrentThread())) {
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    }

    ~ScopedTimeCriticalBoost() noexcept {
        SetThreadPriority(GetCurrentThread(), saved_priority_);
    }

    ScopedTimeCriticalBoost(const ScopedTimeCriticalBoost&) = delete;
    ScopedTimeCriticalBoost& operator=(const ScopedTimeCriticalBoost&) = delete;
    ScopedTimeCriticalBoost(ScopedTimeCriticalBoost&&) = delete;
    ScopedTimeCriticalBoost& operator=(ScopedTimeCriticalBoost&&) = delete;

   private:
    int saved_priority_;
};

}  // namespace yabridge::nspa
