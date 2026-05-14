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
#include "../utils.h"
#include "clap/events.h"

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
inline constexpr uint32_t clap_envelope_flag_transport_valid     = 1u << 0;
inline constexpr uint32_t clap_envelope_flag_input_events_valid  = 1u << 1;

// Producer-side conversion of one ::clap::events::Event to the direct
// envelope slot.  Returns true if the event is a fixed-shape variant
// and was written; false if it carries a variable-shape variant
// (Transport-as-event / MidiSysex) — caller aborts the envelope path
// for the whole block and keeps the entire in_events_ on the bitsery
// path (block-level fallback).
inline bool clap_event_to_direct(const ::clap::events::Event& src,
                                 ClapProcessEventDirect& dst) noexcept {
    namespace pl = ::clap::events::payload;
    return std::visit(
        overload{
            [&](const pl::Note& p) {
                dst.header.size     = p.event.header.size;
                dst.header.time     = p.event.header.time;
                dst.header.space_id = p.event.header.space_id;
                dst.header.type     = p.event.header.type;
                dst.header.flags    = p.event.header.flags;
                dst.payload.note.note_id    = p.event.note_id;
                dst.payload.note.port_index = p.event.port_index;
                dst.payload.note.channel    = p.event.channel;
                dst.payload.note.key        = p.event.key;
                dst.payload.note._pad0      = 0;
                dst.payload.note._pad1      = 0;
                dst.payload.note.velocity   = p.event.velocity;
                std::memset(dst.payload.note._tail_pad, 0,
                            sizeof(dst.payload.note._tail_pad));
                return true;
            },
            [&](const pl::NoteExpression& p) {
                dst.header.size     = p.event.header.size;
                dst.header.time     = p.event.header.time;
                dst.header.space_id = p.event.header.space_id;
                dst.header.type     = p.event.header.type;
                dst.header.flags    = p.event.header.flags;
                dst.payload.note_expression.expression_id = p.event.expression_id;
                dst.payload.note_expression.note_id    = p.event.note_id;
                dst.payload.note_expression.port_index = p.event.port_index;
                dst.payload.note_expression.channel    = p.event.channel;
                dst.payload.note_expression.key        = p.event.key;
                dst.payload.note_expression._pad0      = 0;
                dst.payload.note_expression.value      = p.event.value;
                std::memset(dst.payload.note_expression._tail_pad, 0,
                            sizeof(dst.payload.note_expression._tail_pad));
                return true;
            },
            [&](const pl::ParamValue& p) {
                dst.header.size     = p.event.header.size;
                dst.header.time     = p.event.header.time;
                dst.header.space_id = p.event.header.space_id;
                dst.header.type     = p.event.header.type;
                dst.header.flags    = p.event.header.flags;
                dst.payload.param_value_or_mod.param_id   = p.event.param_id;
                dst.payload.param_value_or_mod._pad0      = 0;
                dst.payload.param_value_or_mod.cookie     =
                    reinterpret_cast<uint64_t>(p.event.cookie);
                dst.payload.param_value_or_mod.note_id    = p.event.note_id;
                dst.payload.param_value_or_mod.port_index = p.event.port_index;
                dst.payload.param_value_or_mod.channel    = p.event.channel;
                dst.payload.param_value_or_mod.key        = p.event.key;
                dst.payload.param_value_or_mod._pad1      = 0;
                dst.payload.param_value_or_mod._pad2      = 0;
                dst.payload.param_value_or_mod.value_or_amount = p.event.value;
                std::memset(dst.payload.param_value_or_mod._tail_pad, 0,
                            sizeof(dst.payload.param_value_or_mod._tail_pad));
                return true;
            },
            [&](const pl::ParamMod& p) {
                dst.header.size     = p.event.header.size;
                dst.header.time     = p.event.header.time;
                dst.header.space_id = p.event.header.space_id;
                dst.header.type     = p.event.header.type;
                dst.header.flags    = p.event.header.flags;
                dst.payload.param_value_or_mod.param_id   = p.event.param_id;
                dst.payload.param_value_or_mod._pad0      = 0;
                dst.payload.param_value_or_mod.cookie     =
                    reinterpret_cast<uint64_t>(p.event.cookie);
                dst.payload.param_value_or_mod.note_id    = p.event.note_id;
                dst.payload.param_value_or_mod.port_index = p.event.port_index;
                dst.payload.param_value_or_mod.channel    = p.event.channel;
                dst.payload.param_value_or_mod.key        = p.event.key;
                dst.payload.param_value_or_mod._pad1      = 0;
                dst.payload.param_value_or_mod._pad2      = 0;
                dst.payload.param_value_or_mod.value_or_amount = p.event.amount;
                std::memset(dst.payload.param_value_or_mod._tail_pad, 0,
                            sizeof(dst.payload.param_value_or_mod._tail_pad));
                return true;
            },
            [&](const pl::ParamGesture& p) {
                dst.header.size     = p.event.header.size;
                dst.header.time     = p.event.header.time;
                dst.header.space_id = p.event.header.space_id;
                dst.header.type     = p.event.header.type;
                dst.header.flags    = p.event.header.flags;
                dst.payload.param_gesture.param_id = p.event.param_id;
                std::memset(dst.payload.param_gesture._tail_pad, 0,
                            sizeof(dst.payload.param_gesture._tail_pad));
                return true;
            },
            [](const pl::Transport&) { return false; },
            [&](const pl::Midi& p) {
                dst.header.size     = p.event.header.size;
                dst.header.time     = p.event.header.time;
                dst.header.space_id = p.event.header.space_id;
                dst.header.type     = p.event.header.type;
                dst.header.flags    = p.event.header.flags;
                dst.payload.midi.port_index = p.event.port_index;
                dst.payload.midi.data[0]    = p.event.data[0];
                dst.payload.midi.data[1]    = p.event.data[1];
                dst.payload.midi.data[2]    = p.event.data[2];
                std::memset(dst.payload.midi._tail_pad, 0,
                            sizeof(dst.payload.midi._tail_pad));
                return true;
            },
            [](const pl::MidiSysex&) { return false; },
            [&](const pl::Midi2& p) {
                dst.header.size     = p.event.header.size;
                dst.header.time     = p.event.header.time;
                dst.header.space_id = p.event.header.space_id;
                dst.header.type     = p.event.header.type;
                dst.header.flags    = p.event.header.flags;
                dst.payload.midi2.port_index = p.event.port_index;
                dst.payload.midi2._pad0      = 0;
                dst.payload.midi2.data[0]    = p.event.data[0];
                dst.payload.midi2.data[1]    = p.event.data[1];
                dst.payload.midi2.data[2]    = p.event.data[2];
                dst.payload.midi2.data[3]    = p.event.data[3];
                std::memset(dst.payload.midi2._tail_pad, 0,
                            sizeof(dst.payload.midi2._tail_pad));
                return true;
            }},
        src.payload);
}

// Consumer-side conversion.  Materializes a ::clap::events::Event from
// the direct envelope slot.  Dispatches on header.type for the
// appropriate variant.  Unknown types leave the event default-
// constructed (Note variant zero-initialized) — should never happen
// given version-match on attach.
inline void clap_event_from_direct(const ClapProcessEventDirect& src,
                                   ::clap::events::Event& dst) noexcept {
    namespace pl = ::clap::events::payload;
    auto fill_header = [&](clap_event_header_t& h) {
        h.size     = src.header.size;
        h.time     = src.header.time;
        h.space_id = src.header.space_id;
        h.type     = src.header.type;
        h.flags    = src.header.flags;
    };
    switch (src.header.type) {
        case CLAP_EVENT_NOTE_ON:
        case CLAP_EVENT_NOTE_OFF:
        case CLAP_EVENT_NOTE_CHOKE:
        case CLAP_EVENT_NOTE_END: {
            pl::Note p{};
            fill_header(p.event.header);
            p.event.note_id    = src.payload.note.note_id;
            p.event.port_index = src.payload.note.port_index;
            p.event.channel    = src.payload.note.channel;
            p.event.key        = src.payload.note.key;
            p.event.velocity   = src.payload.note.velocity;
            dst.payload = std::move(p);
            break;
        }
        case CLAP_EVENT_NOTE_EXPRESSION: {
            pl::NoteExpression p{};
            fill_header(p.event.header);
            p.event.expression_id = src.payload.note_expression.expression_id;
            p.event.note_id    = src.payload.note_expression.note_id;
            p.event.port_index = src.payload.note_expression.port_index;
            p.event.channel    = src.payload.note_expression.channel;
            p.event.key        = src.payload.note_expression.key;
            p.event.value      = src.payload.note_expression.value;
            dst.payload = std::move(p);
            break;
        }
        case CLAP_EVENT_PARAM_VALUE: {
            pl::ParamValue p{};
            fill_header(p.event.header);
            p.event.param_id   = src.payload.param_value_or_mod.param_id;
            p.event.cookie     = reinterpret_cast<void*>(
                src.payload.param_value_or_mod.cookie);
            p.event.note_id    = src.payload.param_value_or_mod.note_id;
            p.event.port_index = src.payload.param_value_or_mod.port_index;
            p.event.channel    = src.payload.param_value_or_mod.channel;
            p.event.key        = src.payload.param_value_or_mod.key;
            p.event.value      = src.payload.param_value_or_mod.value_or_amount;
            dst.payload = std::move(p);
            break;
        }
        case CLAP_EVENT_PARAM_MOD: {
            pl::ParamMod p{};
            fill_header(p.event.header);
            p.event.param_id   = src.payload.param_value_or_mod.param_id;
            p.event.cookie     = reinterpret_cast<void*>(
                src.payload.param_value_or_mod.cookie);
            p.event.note_id    = src.payload.param_value_or_mod.note_id;
            p.event.port_index = src.payload.param_value_or_mod.port_index;
            p.event.channel    = src.payload.param_value_or_mod.channel;
            p.event.key        = src.payload.param_value_or_mod.key;
            p.event.amount     = src.payload.param_value_or_mod.value_or_amount;
            dst.payload = std::move(p);
            break;
        }
        case CLAP_EVENT_PARAM_GESTURE_BEGIN:
        case CLAP_EVENT_PARAM_GESTURE_END: {
            pl::ParamGesture p{};
            fill_header(p.event.header);
            p.event.param_id = src.payload.param_gesture.param_id;
            dst.payload = std::move(p);
            break;
        }
        case CLAP_EVENT_MIDI: {
            pl::Midi p{};
            fill_header(p.event.header);
            p.event.port_index = src.payload.midi.port_index;
            p.event.data[0]    = src.payload.midi.data[0];
            p.event.data[1]    = src.payload.midi.data[1];
            p.event.data[2]    = src.payload.midi.data[2];
            dst.payload = std::move(p);
            break;
        }
        case CLAP_EVENT_MIDI2: {
            pl::Midi2 p{};
            fill_header(p.event.header);
            p.event.port_index = src.payload.midi2.port_index;
            p.event.data[0]    = src.payload.midi2.data[0];
            p.event.data[1]    = src.payload.midi2.data[1];
            p.event.data[2]    = src.payload.midi2.data[2];
            p.event.data[3]    = src.payload.midi2.data[3];
            dst.payload = std::move(p);
            break;
        }
        default:
            // Unknown type — leave dst.payload default-constructed.
            // Should never happen given version-match on attach.
            break;
    }
}

}  // namespace yabridge::nspa
