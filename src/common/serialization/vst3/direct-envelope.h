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

#include <pluginterfaces/vst/ivstevents.h>
#include <pluginterfaces/vst/ivstparameterchanges.h>
#include <pluginterfaces/vst/ivstprocesscontext.h>

#include "../../audio-control-shm.h"
#include "../../utils.h"
#include "event-list.h"
#include "param-value-queue.h"
#include "parameter-changes.h"

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
inline constexpr uint32_t vst3_envelope_flag_process_context_valid    = 1u << 0;
inline constexpr uint32_t vst3_envelope_flag_input_events_valid       = 1u << 1;
inline constexpr uint32_t vst3_envelope_flag_input_param_changes_valid = 1u << 2;

// Flag bit definitions for Vst3ProcessReplyEnvelope::flags (P5).
inline constexpr uint32_t vst3_reply_envelope_flag_output_events_valid        = 1u << 0;
inline constexpr uint32_t vst3_reply_envelope_flag_output_param_changes_valid = 1u << 1;

// Producer-side conversion of a single YaParamValueQueue to the direct
// envelope slot.  Returns true if the queue's point count fits within
// max_param_points_per_queue and was written; false otherwise — the
// caller then aborts the envelope path for the entire block-level
// input_parameter_changes_ and leaves it on the bitsery path.
inline bool param_queue_to_direct(const YaParamValueQueue& src,
                                  ProcessParamQueueDirect& dst) noexcept {
    const auto& points = src.queue_ref();
    if (points.size() > max_param_points_per_queue) {
        return false;
    }
    dst.parameter_id = src.parameter_id_;
    dst.point_count  = static_cast<uint32_t>(points.size());
    dst._hdr_pad[0]  = 0;
    dst._hdr_pad[1]  = 0;
    for (size_t i = 0; i < points.size(); i++) {
        dst.points[i].sample_offset = points[i].first;
        dst.points[i]._pad          = 0;
        dst.points[i].value         = points[i].second;
    }
    return true;
}

// Consumer-side conversion of a direct envelope queue slot into the
// caller-allocated YaParamValueQueue.  Caller is responsible for
// allocating the queue in YaParameterChanges via addParameterData()
// first (which sets parameter_id and clears the points vector); this
// function appends points via the standard addPoint() API.  Returns
// true on success, false if point_count exceeds the envelope cap
// (defensive — should never trip given version-match on attach).
inline bool param_queue_from_direct(const ProcessParamQueueDirect& src,
                                    Steinberg::Vst::IParamValueQueue&
                                        dst) noexcept {
    if (src.point_count > max_param_points_per_queue) {
        return false;
    }
    int32 ignored_index = 0;
    for (uint32_t i = 0; i < src.point_count; i++) {
        dst.addPoint(src.points[i].sample_offset,
                     src.points[i].value, ignored_index);
    }
    return true;
}

// Producer-side conversion of a single YaEvent to the direct-struct
// envelope slot.  Returns true if the event was a fixed-shape variant
// and was written; false if it carries a variable-payload variant
// (DataEvent / NoteExpressionTextEvent / ChordEvent / ScaleEvent) —
// the caller must then abort the envelope path for the entire block
// and keep input_events_ on the bitsery path (block-level fallback).
//
// On false return, `dst` may have been partially populated.  Callers
// should treat partial writes as invalid and refrain from setting the
// envelope's input_events_valid flag.
inline bool yaevent_to_direct(const YaEvent& src,
                              ProcessEventDirect& dst) noexcept {
    dst.bus_index     = src.bus_index;
    dst.sample_offset = src.sample_offset;
    dst.ppq_position  = src.ppq_position;
    dst.flags         = src.flags;
    dst._pad0         = 0;
    return std::visit(
        overload{
            [&](const Steinberg::Vst::NoteOnEvent& e) {
                dst.event_kind = Steinberg::Vst::Event::kNoteOnEvent;
                dst.payload.note_on.channel   = e.channel;
                dst.payload.note_on.pitch     = e.pitch;
                dst.payload.note_on.tuning    = e.tuning;
                dst.payload.note_on.velocity  = e.velocity;
                dst.payload.note_on.length    = e.length;
                dst.payload.note_on.note_id   = e.noteId;
                dst.payload.note_on._tail_pad = 0;
                return true;
            },
            [&](const Steinberg::Vst::NoteOffEvent& e) {
                dst.event_kind = Steinberg::Vst::Event::kNoteOffEvent;
                dst.payload.note_off.channel   = e.channel;
                dst.payload.note_off.pitch     = e.pitch;
                dst.payload.note_off.velocity  = e.velocity;
                dst.payload.note_off.note_id   = e.noteId;
                dst.payload.note_off.tuning    = e.tuning;
                dst.payload.note_off._tail_pad = 0;
                return true;
            },
            [](const YaDataEvent&) { return false; },
            [&](const Steinberg::Vst::PolyPressureEvent& e) {
                dst.event_kind = Steinberg::Vst::Event::kPolyPressureEvent;
                dst.payload.poly_pressure.channel  = e.channel;
                dst.payload.poly_pressure.pitch    = e.pitch;
                dst.payload.poly_pressure.pressure = e.pressure;
                dst.payload.poly_pressure.note_id  = e.noteId;
                std::memset(dst.payload.poly_pressure._tail_pad, 0,
                            sizeof(dst.payload.poly_pressure._tail_pad));
                return true;
            },
            [&](const Steinberg::Vst::NoteExpressionValueEvent& e) {
                dst.event_kind =
                    Steinberg::Vst::Event::kNoteExpressionValueEvent;
                dst.payload.note_expression_value.type_id   = e.typeId;
                dst.payload.note_expression_value.note_id   = e.noteId;
                dst.payload.note_expression_value.value     = e.value;
                dst.payload.note_expression_value._tail_pad = 0;
                return true;
            },
            [](const YaNoteExpressionTextEvent&) { return false; },
            [](const YaChordEvent&) { return false; },
            [](const YaScaleEvent&) { return false; },
            [&](const Steinberg::Vst::LegacyMIDICCOutEvent& e) {
                dst.event_kind =
                    Steinberg::Vst::Event::kLegacyMIDICCOutEvent;
                dst.payload.legacy_midi_cc_out.control_number =
                    e.controlNumber;
                dst.payload.legacy_midi_cc_out.channel = e.channel;
                dst.payload.legacy_midi_cc_out.value   = e.value;
                dst.payload.legacy_midi_cc_out.value2  = e.value2;
                std::memset(
                    dst.payload.legacy_midi_cc_out._tail_pad, 0,
                    sizeof(dst.payload.legacy_midi_cc_out._tail_pad));
                return true;
            }},
        src.payload);
}

// Consumer-side conversion of a direct-struct envelope slot to a
// YaEvent.  Symmetric with yaevent_to_direct; only handles the
// fixed-shape variants the producer would have written.  Unknown
// event_kind values leave the payload variant in its default-
// constructed state (NoteOnEvent zero-initialized) — this should
// never happen given the version-match check on attach, but is the
// defensive path.
inline void yaevent_from_direct(const ProcessEventDirect& src,
                                YaEvent& dst) noexcept {
    dst.bus_index     = src.bus_index;
    dst.sample_offset = src.sample_offset;
    dst.ppq_position  = src.ppq_position;
    dst.flags         = src.flags;
    switch (src.event_kind) {
        case Steinberg::Vst::Event::kNoteOnEvent: {
            Steinberg::Vst::NoteOnEvent e{};
            e.channel  = src.payload.note_on.channel;
            e.pitch    = src.payload.note_on.pitch;
            e.tuning   = src.payload.note_on.tuning;
            e.velocity = src.payload.note_on.velocity;
            e.length   = src.payload.note_on.length;
            e.noteId   = src.payload.note_on.note_id;
            dst.payload = e;
            break;
        }
        case Steinberg::Vst::Event::kNoteOffEvent: {
            Steinberg::Vst::NoteOffEvent e{};
            e.channel  = src.payload.note_off.channel;
            e.pitch    = src.payload.note_off.pitch;
            e.velocity = src.payload.note_off.velocity;
            e.noteId   = src.payload.note_off.note_id;
            e.tuning   = src.payload.note_off.tuning;
            dst.payload = e;
            break;
        }
        case Steinberg::Vst::Event::kPolyPressureEvent: {
            Steinberg::Vst::PolyPressureEvent e{};
            e.channel  = src.payload.poly_pressure.channel;
            e.pitch    = src.payload.poly_pressure.pitch;
            e.pressure = src.payload.poly_pressure.pressure;
            e.noteId   = src.payload.poly_pressure.note_id;
            dst.payload = e;
            break;
        }
        case Steinberg::Vst::Event::kNoteExpressionValueEvent: {
            Steinberg::Vst::NoteExpressionValueEvent e{};
            e.typeId = src.payload.note_expression_value.type_id;
            e.noteId = src.payload.note_expression_value.note_id;
            e.value  = src.payload.note_expression_value.value;
            dst.payload = e;
            break;
        }
        case Steinberg::Vst::Event::kLegacyMIDICCOutEvent: {
            Steinberg::Vst::LegacyMIDICCOutEvent e{};
            e.controlNumber = src.payload.legacy_midi_cc_out.control_number;
            e.channel       = src.payload.legacy_midi_cc_out.channel;
            e.value         = src.payload.legacy_midi_cc_out.value;
            e.value2        = src.payload.legacy_midi_cc_out.value2;
            dst.payload = e;
            break;
        }
        default:
            // Unknown / malformed event_kind.  Leave payload at its
            // default-constructed state.  Should never happen given
            // version-match on attach.
            break;
    }
}

}  // namespace yabridge::nspa
