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

#include "orphan-cleanup.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <climits>
#include <mutex>
#include <string>
#include <string_view>
#include <system_error>

#include <unistd.h>

#include <ghc/filesystem.hpp>

#include "../common/communication/common.h"
#include "../common/utils.h"

namespace fs = ghc::filesystem;

namespace yabridge::nspa {

namespace {

constexpr std::string_view yabridge_prefix = "yabridge-";
constexpr std::string_view pid_sentinel    = "-pid";

// Parse the owner PID out of a yabridge-prefixed filename.  Names look
// like:
//
//   yabridge-PluginName-RANDOM-pidNNNN
//   yabridge-PluginName-RANDOM-pidNNNN-1            (AudioShmBuffer)
//   yabridge-PluginName-RANDOM-pidNNNN-vst3-0.audio_ctl
//
// Plugin names may contain '-' (e.g. "ACE(x64)" sanitized to
// "ACE_x64_"), so we anchor on the `-pid` sentinel rather than
// positional dash counting.  Returns 0 if no parseable PID is found.
pid_t parse_owner_pid(std::string_view name) noexcept {
    if (!name.starts_with(yabridge_prefix)) {
        return 0;
    }
    const auto pid_pos = name.find(pid_sentinel);
    if (pid_pos == std::string_view::npos) {
        return 0;
    }
    const auto digit_start = pid_pos + pid_sentinel.size();
    auto digit_end = digit_start;
    while (digit_end < name.size() &&
           std::isdigit(static_cast<unsigned char>(name[digit_end]))) {
        digit_end++;
    }
    if (digit_end == digit_start) {
        return 0;
    }

    // Convert digits to int.  Use a bounded parse to avoid overflow on a
    // malformed filename.
    pid_t pid = 0;
    for (auto i = digit_start; i < digit_end; i++) {
        const int d = name[i] - '0';
        if (pid > (INT_MAX - d) / 10) {
            return 0;  // overflow — implausible PID
        }
        pid = pid * 10 + d;
    }
    return pid;
}

// Reuse the same liveness test yabridge's existing `pid_running` uses
// (src/common/process.cpp).  Reimplemented locally here to avoid
// pulling that whole TU as a dep of the orphan-cleanup unit.
bool pid_alive(pid_t pid) noexcept {
    if (pid <= 0) return false;
    std::error_code ec;
    fs::canonical("/proc/" + std::to_string(pid) + "/exe", ec);
    return !ec || ec.value() == EACCES;
}

// Best-effort unlink — log nothing.  Failure modes are all benign:
// ENOENT (someone else cleaned it concurrently), EBUSY (someone has it
// mapped), EACCES (race with permissions change).  None warrant a
// diagnostic from a one-shot startup pass.
void try_unlink(const fs::path& p) noexcept {
    std::error_code ec;
    if (fs::is_directory(p, ec)) {
        fs::remove_all(p, ec);
    } else {
        fs::remove(p, ec);
    }
}

// Scan a single directory for yabridge-prefixed entries with dead-PID
// sentinels.  `dir` typically `/dev/shm` or `${XDG_RUNTIME_DIR}`.
void clean_orphans_in(const fs::path& dir) noexcept {
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) {
        return;
    }
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (ec) {
            break;
        }
        const auto name = entry.path().filename().string();
        if (!name.starts_with(yabridge_prefix)) {
            continue;
        }
        const pid_t owner = parse_owner_pid(name);
        if (owner == 0) {
            // No sentinel — this is an old-format file from a yabridge
            // build that predates the PID-tagging.  Leave it alone; the
            // user can clean those manually if they accumulate.  After
            // a wider rollout we can switch to age-based fallback here.
            continue;
        }
        if (pid_alive(owner)) {
            continue;
        }
        try_unlink(entry.path());
    }
}

}  // namespace

void clean_orphan_yabridge_state() noexcept {
    static std::once_flag once;
    std::call_once(once, [] {
        // Two locations where yabridge artefacts accumulate:
        //
        //   1. The temp directory that hosts socket dirs.  Defaults to
        //      ${XDG_RUNTIME_DIR} or /tmp.  Resolved by the same helper
        //      `get_temporary_directory()` that `generate_endpoint_base`
        //      uses, so the two stay in sync.
        //   2. /dev/shm — global tmpfs where shm_open() lands its files.
        clean_orphans_in(get_temporary_directory());
        clean_orphans_in("/dev/shm");
    });
}

}  // namespace yabridge::nspa
