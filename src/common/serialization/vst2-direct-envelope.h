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

// VST2 direct-struct envelope conversion helpers.  Bridges the
// vestige `VstTimeInfo` class layout on each side of the L2 region
// to the ABI-stable VstTimeInfoDirect wire format defined in
// audio-control-shm.h.
//
// Same shape as the VST3 ProcessContext helpers
// (common/serialization/vst3/direct-envelope.h): inline field-by-field
// copy.  No SIMD — these structs are too small to amortize anything
// vector ops would provide.

#include <vestige/aeffectx.h>

#include "../audio-control-shm.h"

namespace yabridge::nspa {

// Producer-side conversion.
inline void vst_time_info_to_direct(
    const VstTimeInfo& src,
    VstTimeInfoDirect& dst) noexcept {
    dst.sample_pos           = src.samplePos;
    dst.sample_rate          = src.sampleRate;
    dst.nano_seconds         = src.nanoSeconds;
    dst.ppq_pos              = src.ppqPos;
    dst.tempo                = src.tempo;
    dst.bar_start_pos        = src.barStartPos;
    dst.cycle_start_pos      = src.cycleStartPos;
    dst.cycle_end_pos        = src.cycleEndPos;
    dst.time_sig_numerator   = src.timeSigNumerator;
    dst.time_sig_denominator = src.timeSigDenominator;
    std::memcpy(dst.empty3, src.empty3, sizeof(dst.empty3));
    dst.flags                = src.flags;
}

// Consumer-side conversion.
inline void vst_time_info_from_direct(
    const VstTimeInfoDirect& src,
    VstTimeInfo& dst) noexcept {
    dst.samplePos          = src.sample_pos;
    dst.sampleRate         = src.sample_rate;
    dst.nanoSeconds        = src.nano_seconds;
    dst.ppqPos             = src.ppq_pos;
    dst.tempo              = src.tempo;
    dst.barStartPos        = src.bar_start_pos;
    dst.cycleStartPos      = src.cycle_start_pos;
    dst.cycleEndPos        = src.cycle_end_pos;
    dst.timeSigNumerator   = src.time_sig_numerator;
    dst.timeSigDenominator = src.time_sig_denominator;
    std::memcpy(dst.empty3, src.empty3, sizeof(dst.empty3));
    dst.flags              = src.flags;
}

// Flag bit definitions for Vst2ProcessEnvelope::flags.
inline constexpr uint32_t vst2_envelope_flag_time_info_valid = 1u << 0;

}  // namespace yabridge::nspa
