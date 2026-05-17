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
// Three patterns:
//
//   set_thread_time_critical()        — set this thread to TIME_CRITICAL
//                                       and leave it there. Use ONLY
//                                       for the genuine audio worker
//                                       threads (process_replacing
//                                       handler, per-instance audio
//                                       handlers). Maps to SCHED_FIFO
//                                       at NSPA_RT_PRIO (audio band).
//
//   set_thread_realtime_idle()        — set this thread to
//                                       THREAD_PRIORITY_IDLE inside
//                                       REALTIME_PRIORITY_CLASS (the
//                                       lowest band of the Win32 RT
//                                       class).  Named with the
//                                       "realtime_" prefix to avoid
//                                       confusion with Linux's
//                                       SCHED_IDLE policy (which is
//                                       the opposite — the lowest
//                                       non-RT band).  Still
//                                       SCHED_FIFO under NSPA, so
//                                       spawned children inherit a
//                                       real-time policy and
//                                       pthread_create RT requests
//                                       succeed — but well below the
//                                       audio band so the dispatch /
//                                       control / parameters loops
//                                       don't compete with the audio
//                                       thread or starve the desktop.
//                                       Use for thread-entry promotion
//                                       on non-audio dispatcher loops.
//
//   ScopedRealtimeIdleBoost           — RAII: save current priority,
//                                       set to THREAD_PRIORITY_IDLE,
//                                       restore on scope exit. Use for
//                                       bracketed calls into the
//                                       plugin (plugin construction,
//                                       LoadLibrary, IPluginBase::
//                                       initialize, etc.) where
//                                       spawned worker threads must
//                                       inherit RT but the caller's
//                                       prio should not be left
//                                       elevated. IDLE inside RT class
//                                       is enough to satisfy the RT
//                                       inheritance check — no need to
//                                       run the bracketed code at
//                                       audio-equivalent priority.

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

// Promote the calling thread to THREAD_PRIORITY_IDLE.  Under
// REALTIME_PRIORITY_CLASS (yabridge's wine-host process priority class,
// set in host.cpp:107) this lands at the lowest band of the Win32 RT
// class — NT 16 — which NSPA maps to SCHED_FIFO at the bottom of the
// audio range (e.g. FIFO@65 with NSPA_RT_PRIO=80).  Still SCHED_FIFO,
// so pthread_create inheritance grants children RT and the kernel
// accepts SCHED_FIFO requests from this thread.  But well below the
// audio callback band (NT 31 / FIFO@80), so dispatch / control /
// parameters loops don't compete with audio threads or starve the
// desktop during heavy plugin init.
//
// Name disambiguates from Linux's SCHED_IDLE policy (which is the
// opposite — lowest non-RT band).
inline void set_thread_realtime_idle() noexcept {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);
}

// Restore the calling thread to THREAD_PRIORITY_NORMAL. Used by code
// paths that previously called set_realtime_priority(false), which
// directly demoted to SCHED_OTHER. Routing through SetThreadPriority
// keeps NSPA's process-class-aware nice mapping in effect.
inline void set_thread_normal() noexcept {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
}

// RAII helper: on construction, save the current thread priority and
// promote to THREAD_PRIORITY_IDLE (NT 16 inside REALTIME_PRIORITY_CLASS
// — the lowest band of the Win32 RT class, still SCHED_FIFO under
// NSPA but well below the audio band).  On destruction, restore the
// saved value.
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
// effSetSampleRate/effSetBlockSize) spawn worker threads internally.
// Those threads inherit the creator's scheduling policy via
// pthread_create's default PTHREAD_INHERIT_SCHED.  By boosting the
// caller into the RT class before the plugin call, we ensure spawned
// workers come up SCHED_FIFO and pthread_create's RT inheritance
// check passes.  THREAD_PRIORITY_IDLE (rather than TIME_CRITICAL) is
// enough — the bracket is about giving children an RT parent, not
// about running the bracketed plugin code at audio priority.  Heavy
// init (LoadLibrary, preset enumeration, sample loading) used to
// freeze the desktop for ~1s when bracketed at TIME_CRITICAL because
// it competed with the actual audio callback thread.
//
// Plugin worker threads that genuinely need audio-band priority
// (their own audio callback thread) explicitly call
// SetThreadPriority(TIME_CRITICAL) themselves — they don't rely on
// inheriting the bracketed priority.
//
// Also used to bracket LoadLibrary itself (see load_library_rt below).
// The plugin's DllMain runs synchronously inside LoadLibrary, and some
// plugins (notably u-he's VST3s — Zebra2, ACE, etc.) spawn
// boost::thread workers during PROCESS_ATTACH static init. If the
// caller is outside the RT class and the plugin's pthread_create asks
// for SCHED_FIFO, the kernel rejects the request, boost::thread
// throws thread_resource_error, and the plugin dies before reaching
// its normal entry point.
class ScopedRealtimeIdleBoost {
   public:
    ScopedRealtimeIdleBoost() noexcept
        : saved_priority_(GetThreadPriority(GetCurrentThread())) {
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);
    }

    ~ScopedRealtimeIdleBoost() noexcept {
        SetThreadPriority(GetCurrentThread(), saved_priority_);
    }

    ScopedRealtimeIdleBoost(const ScopedRealtimeIdleBoost&) = delete;
    ScopedRealtimeIdleBoost& operator=(const ScopedRealtimeIdleBoost&) = delete;
    ScopedRealtimeIdleBoost(ScopedRealtimeIdleBoost&&) = delete;
    ScopedRealtimeIdleBoost& operator=(ScopedRealtimeIdleBoost&&) = delete;

   private:
    int saved_priority_;
};

// LoadLibrary wrapped in a ScopedRealtimeIdleBoost so plugins which
// spawn boost::thread / std::thread workers during DllMain
// PROCESS_ATTACH or static-initializer code (u-he VST3s among others)
// see an RT-class parent thread and the kernel grants their
// pthread_create RT requests. Used as the member-initializer
// expression for the plugin handle on each bridge.
//
// Returning the result from a body block via a lambda is the only way
// to keep this an in-line expression while still scoping the boost
// guard to the LoadLibrary call alone — we don't want the wrapping
// bridge's other member inits running at elevated priority.
inline HMODULE load_library_rt(const char* path) noexcept {
    ScopedRealtimeIdleBoost boost;
    // Explicit `LoadLibraryA` rather than the `LoadLibrary` macro: the
    // macro resolves to `LoadLibraryW` under UNICODE builds and the
    // bridge call sites pass a `const char*` (the converted DOS path
    // from `to_dos_path`), so the explicit ANSI form keeps the signature
    // stable across UNICODE configurations.
    return LoadLibraryA(path);
}

}  // namespace yabridge::nspa
