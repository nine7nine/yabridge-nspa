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

// CLAP direct-struct envelope conversion helpers.  Bridges the
// `clap_event_transport_t` POD on each side of the L2 region to the
// ABI-stable ClapTransportDirect wire format defined in
// audio-control-shm.h.
//
// Same pattern as the VST3 ProcessContext / VST2 VstTimeInfo helpers
// (common/serialization/{vst3,vst2}-direct-envelope.h).  Inline
// field-by-field copy.

#include <clap/events.h>

#include "../audio-control-shm.h"

namespace yabridge::nspa {

// Producer-side conversion.
inline void clap_transport_to_direct(
    const clap_event_transport_t& src,
    ClapTransportDirect& dst) noexcept {
    dst.header.size               = src.header.size;
    dst.header.time               = src.header.time;
    dst.header.space_id           = src.header.space_id;
    dst.header.type               = src.header.type;
    dst.header.flags              = src.header.flags;
    dst.flags                     = src.flags;
    dst._pad0                     = 0;
    dst.song_pos_beats            = src.song_pos_beats;
    dst.song_pos_seconds          = src.song_pos_seconds;
    dst.tempo                     = src.tempo;
    dst.tempo_inc                 = src.tempo_inc;
    dst.loop_start_beats          = src.loop_start_beats;
    dst.loop_end_beats            = src.loop_end_beats;
    dst.loop_start_seconds        = src.loop_start_seconds;
    dst.loop_end_seconds          = src.loop_end_seconds;
    dst.bar_start                 = src.bar_start;
    dst.bar_number                = src.bar_number;
    dst.tsig_num                  = src.tsig_num;
    dst.tsig_denom                = src.tsig_denom;
    dst._trail_pad                = 0;
}

// Consumer-side conversion.
inline void clap_transport_from_direct(
    const ClapTransportDirect& src,
    clap_event_transport_t& dst) noexcept {
    dst.header.size       = src.header.size;
    dst.header.time       = src.header.time;
    dst.header.space_id   = src.header.space_id;
    dst.header.type       = src.header.type;
    dst.header.flags      = src.header.flags;
    dst.flags             = src.flags;
    dst.song_pos_beats    = src.song_pos_beats;
    dst.song_pos_seconds  = src.song_pos_seconds;
    dst.tempo             = src.tempo;
    dst.tempo_inc         = src.tempo_inc;
    dst.loop_start_beats  = src.loop_start_beats;
    dst.loop_end_beats    = src.loop_end_beats;
    dst.loop_start_seconds = src.loop_start_seconds;
    dst.loop_end_seconds  = src.loop_end_seconds;
    dst.bar_start         = src.bar_start;
    dst.bar_number        = src.bar_number;
    dst.tsig_num          = src.tsig_num;
    dst.tsig_denom        = src.tsig_denom;
}

// Flag bit definitions for ClapProcessEnvelope::flags.
inline constexpr uint32_t clap_envelope_flag_transport_valid = 1u << 0;

}  // namespace yabridge::nspa
