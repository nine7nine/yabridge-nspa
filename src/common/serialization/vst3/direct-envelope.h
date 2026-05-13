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

// VST3 direct-struct envelope conversion helpers.  Bridges the
// Steinberg::Vst::ProcessContext ABI on each side of the L2 region
// to the ABI-stable ProcessContextDirect wire format defined in
// audio-control-shm.h.
//
// Used by:
//   - Producer (plugin-lib, Linux-native gcc) writes the envelope
//     before calling audio_control_send_and_wait when
//     envelope_active() is true.  Writes happen-before the
//     release-store of state inside send_and_wait — visible to the
//     consumer via the state.load(acquire) → RequestReady observed.
//   - Consumer (wine-host, winegcc PE side) reads the envelope after
//     the bitsery decode of the request, when envelope_active() is
//     true, to override the (parallel-written) bitsery value.  The
//     bitsery path is preserved as fallback.

#include <pluginterfaces/vst/ivstprocesscontext.h>

#include "../../audio-control-shm.h"

namespace yabridge::nspa {

// Producer-side conversion.  Field-by-field copy with explicit width
// types — safe across the producer / consumer ABI boundary.
inline void process_context_to_direct(
    const Steinberg::Vst::ProcessContext& src,
    ProcessContextDirect& dst) noexcept {
    dst.state                   = src.state;
    dst._pad0                   = 0;
    dst.sample_rate             = src.sampleRate;
    dst.project_time_samples    = src.projectTimeSamples;
    dst.system_time             = src.systemTime;
    dst.continous_time_samples  = src.continousTimeSamples;
    dst.project_time_music      = src.projectTimeMusic;
    dst.bar_position_music      = src.barPositionMusic;
    dst.cycle_start_music       = src.cycleStartMusic;
    dst.cycle_end_music         = src.cycleEndMusic;
    dst.tempo                   = src.tempo;
    dst.time_sig_numerator      = src.timeSigNumerator;
    dst.time_sig_denominator    = src.timeSigDenominator;
    dst.chord_key_note          = src.chord.keyNote;
    dst.chord_root_note         = src.chord.rootNote;
    dst.chord_mask              = src.chord.chordMask;
    dst.smpte_offset_subframes  = src.smpteOffsetSubframes;
    dst.frame_rate_fps          = src.frameRate.framesPerSecond;
    dst.frame_rate_flags        = src.frameRate.flags;
    dst.samples_to_next_clock   = src.samplesToNextClock;
    dst._trail_pad              = 0;
}

// Consumer-side conversion.  Mirror of process_context_to_direct.
inline void process_context_from_direct(
    const ProcessContextDirect& src,
    Steinberg::Vst::ProcessContext& dst) noexcept {
    dst.state                = src.state;
    dst.sampleRate           = src.sample_rate;
    dst.projectTimeSamples   = src.project_time_samples;
    dst.systemTime           = src.system_time;
    dst.continousTimeSamples = src.continous_time_samples;
    dst.projectTimeMusic     = src.project_time_music;
    dst.barPositionMusic     = src.bar_position_music;
    dst.cycleStartMusic      = src.cycle_start_music;
    dst.cycleEndMusic        = src.cycle_end_music;
    dst.tempo                = src.tempo;
    dst.timeSigNumerator     = src.time_sig_numerator;
    dst.timeSigDenominator   = src.time_sig_denominator;
    dst.chord.keyNote        = src.chord_key_note;
    dst.chord.rootNote       = src.chord_root_note;
    dst.chord.chordMask      = src.chord_mask;
    dst.smpteOffsetSubframes = src.smpte_offset_subframes;
    dst.frameRate.framesPerSecond = src.frame_rate_fps;
    dst.frameRate.flags      = src.frame_rate_flags;
    dst.samplesToNextClock   = src.samples_to_next_clock;
}

// Flag bit definitions for Vst3ProcessEnvelope::flags.
inline constexpr uint32_t vst3_envelope_flag_process_context_valid = 1u << 0;

}  // namespace yabridge::nspa
