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

#pragma once

// One-shot orphan cleanup of stale yabridge IPC state from crashed prior
// sessions.  Run once per plugin-lib (libyabridge-{vst2,vst3,clap}.so)
// loaded into a DAW process.
//
// Crashed yabridge instances (DAW SIGKILL/SIGSEGV, OOM-kill, host-side
// teardown failures) leave artefacts behind:
//
//   - Socket directories under `${XDG_RUNTIME_DIR}/yabridge-*` (typically
//     `/run/user/<uid>/yabridge-*`).  Cleaned by systemd on user-session
//     end, but accumulate during a session.
//   - Shared-memory regions under `/dev/shm/yabridge-*`.  No automatic
//     OS-level cleanup until reboot.  Most visible — 100+ MB-class
//     audio buffer regions can accumulate over a heavy day of testing.
//
// Identifying orphans is done via a PID sentinel embedded into the
// endpoint base name by `generate_endpoint_base` — every file/dir
// belonging to one yabridge instance ends with `-pid<N>[-suffix]`,
// where N is the plugin-lib's getpid().  At cleanup time, parse the
// sentinel back out and check `/proc/<PID>/exe` for liveness:
//
//   - PID alive → owned by a running DAW, leave alone.
//   - PID dead → orphan from a crashed session, safe to unlink.
//
// Concurrency: multiple DAWs running the cleanup in parallel race on
// `unlink()`, which is idempotent (one wins, others ENOENT).
//
// PID-recycling false negative: if the kernel happens to recycle a
// dead-yabridge PID onto an unrelated live process by the time we
// scan, we'd preserve dead files thinking they're alive.  Very low
// real-world likelihood on Linux with default `pid_max`, and the
// failure mode is conservative (under-clean, not over-clean) — the
// files will get cleaned on the next plugin-lib init when the
// recycled process exits.

namespace yabridge::nspa {

// Idempotent.  Internally guarded by `std::once_flag` so it's safe to
// call from every factory entry point without paying the scan cost
// more than once per .so load.
void clean_orphan_yabridge_state() noexcept;

}  // namespace yabridge::nspa
