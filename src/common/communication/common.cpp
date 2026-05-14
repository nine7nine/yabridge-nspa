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

#include "common.h"

#include <random>
#include <sstream>

#include <unistd.h>

#include "../utils.h"

namespace fs = ghc::filesystem;

/**
 * Used for generating random identifiers.
 */
constexpr char alphanumeric_characters[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

ghc::filesystem::path generate_endpoint_base(const std::string& plugin_name) {
    fs::path temp_directory = get_temporary_directory();

    std::random_device random_device;
    std::mt19937 rng(random_device());
    fs::path candidate_endpoint;
    do {
        std::string random_id;
        std::sample(
            alphanumeric_characters,
            alphanumeric_characters + strlen(alphanumeric_characters) - 1,
            std::back_inserter(random_id), 8, rng);

        // We'll get rid of the file descriptors immediately after accepting the
        // sockets, so putting them inside of a subdirectory would only leave
        // behind an empty directory.
        //
        // NSPA addition: append `-pid<N>` sentinel where N is the plugin-lib's
        // getpid().  Every downstream artifact (socket dir, AudioShmBuffer
        // shm files, L2 audio_ctl shm files) is derived from this base name,
        // so embedding the owner PID once propagates to all of them.  The
        // orphan-state cleanup pass on plugin-lib init (see
        // `clean_orphan_yabridge_state` in src/plugin/utils.cpp) parses the
        // PID back out of the filename and tests `/proc/<PID>` to decide
        // whether a leftover file belongs to a live session or a crashed
        // one.  Using a `pid` sentinel rather than positional digits keeps
        // parsing unambiguous even when suffixes like `-vst3-N.audio_ctl`
        // are appended later.
        std::ostringstream socket_name;
        socket_name << "yabridge-" << plugin_name << "-" << random_id
                    << "-pid" << getpid();

        candidate_endpoint = temp_directory / socket_name.str();
    } while (fs::exists(candidate_endpoint));

    return candidate_endpoint;
}
