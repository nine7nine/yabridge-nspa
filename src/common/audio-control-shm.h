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

// AudioControlShm — process-shared rendezvous region for one audio
// request/reply round-trip, using two pi_mutex + pi_cond pairs (one per
// direction) from the vendored rtpi.h.
//
// This is the L2 transport for the audio-callback hot path. Replaces the
// unix-socket round-trip in libyabridge<->yabridge-host with a userspace
// pi_cond wait. Two semantic wins:
//
//   1. Cross-process priority inheritance. The plugin-lib side (in the
//      DAW process, on the DAW's audio thread) signals the wine-host
//      side; the kernel requeues the waiter onto the PI mutex with PI
//      boost from the signaller. Wine-host worker thread runs at the
//      DAW's effective priority for the duration of the callback, then
//      drops back. No need for yabridge's old 10s polling sched_priority
//      sync — the kernel does it per-callback.
//
//   2. Lower per-callback latency. Stock socket round-trip is sendmsg +
//      kernel buffer + recvmsg + scheduler hops on both ends. Futex
//      round-trip is one wake/wait pair plus one CAS. At small audio
//      buffer sizes (64-128 frames) the saved microseconds matter.
//
// Design history (2026-05-11):
//
//   v1 of this region used a SINGLE pi_mutex + pi_cond pair and held the
//   pi_mutex across the entire round-trip including the plugin
//   processReplacing call on the consumer side. That violated the RT
//   contract (cross-process mutex held during arbitrary plugin code) and
//   was reverted (yabridge commit c88e92b4 on nspa-integration branch).
//
//   v2 (this file) uses two pairs — one for request direction, one for
//   reply direction — and the consumer RELEASES the request pi_mutex
//   BEFORE invoking the plugin handler. Plugin processing happens with
//   no cross-process lock held. The consumer reacquires the reply lock
//   only to publish the reply. Pi_mutex hold spans are µs-bounded.
//
// Architecture (v2):
//
//   Producer (libyabridge plugin-lib side, on the DAW's audio thread):
//
//     pi_mutex_lock(&req_lock)
//     memcpy(request_buf, serialized_request)
//     request_size = N
//     state = REQUEST_READY
//     pi_cond_signal(&req_cv)            // wakes consumer with PI boost
//     pi_mutex_unlock(&req_lock)         // request lock released
//
//     pi_mutex_lock(&reply_lock)
//     while (state != REPLY_READY)
//         pi_cond_wait(&reply_cv)        // sleeps until reply arrives
//     read reply_buf, reply_size
//     state = IDLE
//     pi_mutex_unlock(&reply_lock)
//
//   Consumer (yabridge-host wine-host side, on the audio worker thread):
//
//     pi_mutex_lock(&req_lock)
//     while (state != REQUEST_READY)
//         pi_cond_wait(&req_cv)          // boosted by signaller on wake
//     read request_buf, request_size into local buffer
//     pi_mutex_unlock(&req_lock)         // request lock released
//
//     <process audio callback in plugin DLL — NO cross-process lock>
//
//     pi_mutex_lock(&reply_lock)
//     memcpy(reply_buf, serialized_reply)
//     reply_size = N
//     state = REPLY_READY
//     pi_cond_signal(&reply_cv)          // wakes producer
//     pi_mutex_unlock(&reply_lock)
//
// Shutdown:
//
//   Both producer and consumer check the state atomically under their
//   respective lock. signal_shutdown() flips state to SHUTDOWN and
//   broadcasts both cond vars. Whichever direction the audio thread is
//   currently waiting in, it wakes, observes SHUTDOWN, and throws
//   AudioControlShutdown for the loop to catch.
//
// Lifecycle:
//
//   - Plugin-lib is the CREATOR. AudioControlShm(Create{}, name) does
//     shm_open(O_CREAT|O_EXCL) + ftruncate + mmap(MAP_LOCKED) + init
//     both pi_mutex+pi_cond pairs PSHARED + state = Idle.
//
//   - Wine-host is the PEER. AudioControlShm(Attach{}, name) does
//     shm_open(O_RDWR) + mmap. Does NOT re-init the sync primitives.
//
//   - Creator destructor: pi_*_destroy + munmap + close + shm_unlink.
//   - Peer destructor:    munmap + close (no shm_unlink — creator owns).
//
//   Member declaration order in bridge classes is critical: the
//   AudioControlShm member must be declared so it destructs AFTER any
//   Win32Thread member that uses it. Reverse-of-declaration destruction
//   order applies. The Vst2Bridge fix declares AudioControlShm FIRST.
//
// Crash robustness: pi_mutex with NSPA's rtpi.h doesn't support
// RTPI_MUTEX_ROBUST. If the DAW crashes mid-callback the wine-host
// worker stays blocked. A separate pidfd watchdog (future commit) tears
// the bridge down on DAW death; same defence layer the socket transport
// uses for ECONNRESET.
//
// RT contract:
//
//   - No heap allocation on the audio path. Buffers preallocated at
//     construction (inline storage in SerializationBuffer<64K> members
//     of the bridge classes).
//   - Cross-process mutex hold span is µs-bounded (state transitions
//     and memcpy only — NOT spanning plugin processing).
//   - pi_cond_wait yields (kernel futex syscall, not spin).
//   - PI boost via FUTEX_CMP_REQUEUE_PI on signal — the signaller's
//     effective priority is applied to the woken waiter for the
//     critical section duration.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "rtpi.h"

namespace yabridge::nspa {

// State machine values for the rendezvous slot. Stored as a uint32_t
// atomic in the header. Both directions atomically read this; writes
// happen under the appropriate direction's lock + cond signal.
enum class AudioControlState : uint32_t {
    Idle          = 0,  // No request in flight. Slot is free.
    RequestReady  = 1,  // Producer has written a request; consumer should process.
    ReplyReady    = 2,  // Consumer has written a reply; producer should consume.
    Shutdown      = 3,  // Bridge teardown. Both sides should exit their loops.
};

// Sentinel exception thrown by audio_control_recv_one when it observes
// the Shutdown state. Caught at the audio loop level to break out.
struct AudioControlShutdown : public std::exception {
    const char* what() const noexcept override {
        return "AudioControlShm: shutdown requested";
    }
};

// Maximum bytes a single serialized request or reply can carry through
// the shmem region. Sized to comfortably fit any of the audio request
// structs after bitsery serialization. If a payload overflows this,
// the call site falls back to socket transport for that callback.
constexpr size_t audio_control_buf_size = size_t{64} * 1024;

// Layout version for the L2 region's direct-struct envelope extension.
// Bumped any time the on-shmem envelope layout changes between yabridge
// builds, so peer + creator from different builds can detect mismatch on
// attach and fall back to the bitsery+shmem path transparently.
//
// Version 1 = bitsery payload only (Commit 1 scaffold; no direct-struct
//             extension active).
// Version 2 = VST3 request envelope with ProcessContext direct.
// Version 3 = VST2 request envelope with VstTimeInfo direct.
// Version 4 = CLAP request envelope with clap_event_transport_t direct.
// Version 5 = VST3 request envelope grown with fixed-shape event ring.
// Version 6 = VST3 request envelope grown with parameter queue array.
// Version 7 = CLAP request envelope grown with fixed-shape event ring.
// Version 8 = VST3 reply envelope (output_events_ + output param queues
//             direct in shmem).
// Version 9 = CLAP reply envelope (out_events_ direct) (this commit).
constexpr uint32_t audio_control_layout_version = 9;

// Direct-struct representation of VST3's Steinberg::Vst::ProcessContext.
// Wire format only — producer (plugin-lib) and consumer (wine-host)
// each convert to/from Steinberg::Vst::ProcessContext at their own
// boundary.  Explicit-width fields + natural alignment + flattened
// nested structs (Chord, FrameRate) to keep the layout ABI-stable
// across the Linux-native plugin-lib side and the winegcc-built
// wine-host PE side.  No vst3 SDK header is referenced here.
//
// Field order mirrors Steinberg::Vst::ProcessContext exactly so the
// conversion at each side is straightforward field-by-field copying.
struct ProcessContextDirect {
    uint32_t state;                          // 0   StateAndFlags bitmask
    uint32_t _pad0;                          // 4   align sample_rate to 8
    double   sample_rate;                    // 8
    int64_t  project_time_samples;           // 16
    int64_t  system_time;                    // 24
    int64_t  continous_time_samples;         // 32
    double   project_time_music;             // 40
    double   bar_position_music;             // 48
    double   cycle_start_music;              // 56
    double   cycle_end_music;                // 64
    double   tempo;                          // 72
    int32_t  time_sig_numerator;             // 80
    int32_t  time_sig_denominator;           // 84
    uint8_t  chord_key_note;                 // 88   Chord.keyNote
    uint8_t  chord_root_note;                // 89   Chord.rootNote
    int16_t  chord_mask;                     // 90   Chord.chordMask
    int32_t  smpte_offset_subframes;         // 92
    uint32_t frame_rate_fps;                 // 96   FrameRate.framesPerSecond
    uint32_t frame_rate_flags;               // 100  FrameRate.flags
    int32_t  samples_to_next_clock;          // 104
    uint32_t _trail_pad;                     // 108  align struct size to 8
};
static_assert(sizeof(ProcessContextDirect) == 112,
              "ProcessContextDirect ABI: 112-byte layout sanity check");
static_assert(alignof(ProcessContextDirect) == 8,
              "ProcessContextDirect ABI: 8-byte natural alignment");

// Direct-struct mirror of Steinberg::Vst::Event for the fixed-shape
// variants (NoteOn/NoteOff/PolyPressure/NoteExpressionValue/
// LegacyMIDICCOut).  Variable-payload variants (DataEvent,
// NoteExpressionTextEvent, ChordEvent, ScaleEvent) stay on the bitsery
// path — if a block contains ANY variable variant, the entire input
// event list is kept in bitsery and the envelope event flag is not
// set.  Block-level fallback, no per-event mixing.
//
// 48 bytes, 16-byte aligned.  Header is shared across variants; the
// 24-byte payload union is sized to the largest fixed variant
// (NoteOnEvent: 24 bytes).  Explicit-width primitives + manual padding
// keep the layout ABI-stable across producer (Linux native gcc) and
// consumer (winegcc PE side).  Conversion helpers in
// `common/serialization/vst3/direct-envelope.h`.
struct alignas(16) ProcessEventDirect {
    int32_t  bus_index;          // 0
    int32_t  sample_offset;      // 4
    double   ppq_position;       // 8    Vst::TQuarterNotes (double)
    uint16_t flags;              // 16   Steinberg::Vst::Event::EventFlags
    uint16_t event_kind;         // 18   Steinberg::Vst::Event::EventTypes
    uint32_t _pad0;              // 20   align payload to 24
    union {
        // NoteOnEvent — 24 bytes
        struct {
            int16_t  channel;        // 0
            int16_t  pitch;          // 2
            float    tuning;         // 4
            float    velocity;       // 8
            int32_t  length;         // 12
            int32_t  note_id;        // 16
            uint32_t _tail_pad;      // 20
        } note_on;
        // NoteOffEvent — 24 bytes
        struct {
            int16_t  channel;        // 0
            int16_t  pitch;          // 2
            float    velocity;       // 4
            int32_t  note_id;        // 8
            float    tuning;         // 12
            uint64_t _tail_pad;      // 16
        } note_off;
        // PolyPressureEvent — 24 bytes
        struct {
            int16_t  channel;        // 0
            int16_t  pitch;          // 2
            float    pressure;       // 4
            int32_t  note_id;        // 8
            uint8_t  _tail_pad[12];  // 12
        } poly_pressure;
        // NoteExpressionValueEvent — 24 bytes
        struct {
            int32_t  type_id;        // 0
            int32_t  note_id;        // 4
            double   value;          // 8
            uint64_t _tail_pad;      // 16
        } note_expression_value;
        // LegacyMIDICCOutEvent — 24 bytes
        struct {
            uint8_t  control_number; // 0
            int8_t   channel;        // 1
            int8_t   value;          // 2
            int8_t   value2;         // 3
            uint8_t  _tail_pad[20];  // 4
        } legacy_midi_cc_out;
        uint8_t raw[24];
    } payload;                   // 24 bytes — starts at offset 24
};
static_assert(sizeof(ProcessEventDirect) == 48,
              "ProcessEventDirect ABI: 48-byte layout sanity check");
static_assert(alignof(ProcessEventDirect) == 16,
              "ProcessEventDirect ABI: 16-byte natural alignment");
static_assert(offsetof(ProcessEventDirect, payload) == 24,
              "ProcessEventDirect payload at offset 24");

// Maximum number of fixed-shape events the envelope can carry in one
// block.  Sized for typical MIDI / note-expression density (32-64
// events is heavy in practice); 256 leaves room for synthesizer test
// stress patterns without forcing the bitsery path.
constexpr size_t max_events_per_envelope = 256;

// Direct-struct mirror of one parameter automation point — a
// (sample_offset, value) pair from Steinberg::Vst::IParamValueQueue.
// 16 bytes, 16-byte aligned; pad keeps `value` at natural alignment
// for double on both 32-bit and 64-bit ABIs.
struct alignas(16) ProcessParamPointDirect {
    int32_t  sample_offset;      // 0
    uint32_t _pad;               // 4   align value to 8
    double   value;              // 8
};
static_assert(sizeof(ProcessParamPointDirect) == 16,
              "ProcessParamPointDirect ABI: 16-byte layout sanity check");
static_assert(alignof(ProcessParamPointDirect) == 16,
              "ProcessParamPointDirect ABI: 16-byte natural alignment");

// Maximum number of automation points one parameter queue can carry
// in the envelope.  64 points covers typical Ableton-style automation
// (one envelope move per few samples), per-buffer modulation up to
// once-every-4-samples-on-256-sample-buffers.  Per-sample modulation
// (256 points/buf or denser) falls back to bitsery for that block.
constexpr size_t max_param_points_per_queue = 64;

// One parameter queue: header (param_id + point count) followed by
// the fixed-cap points array.  16-byte aligned; header is 16 bytes so
// the points[] array starts at natural alignment without compiler-
// inserted pad.  Per-queue size: 16 + 64*16 = 1040 bytes.
struct alignas(16) ProcessParamQueueDirect {
    uint32_t parameter_id;       // 0    Steinberg::Vst::ParamID
    uint32_t point_count;        // 4
    uint32_t _hdr_pad[2];        // 8    align points[] to 16
    ProcessParamPointDirect points[max_param_points_per_queue];
};
static_assert(sizeof(ProcessParamQueueDirect) == 16 +
                  max_param_points_per_queue *
                      sizeof(ProcessParamPointDirect),
              "ProcessParamQueueDirect size sanity check");
static_assert(alignof(ProcessParamQueueDirect) == 16,
              "ProcessParamQueueDirect 16-byte natural alignment");

// Maximum number of parameter queues the envelope can carry per
// block.  Typical sessions automate 4-12 params at once; 32 leaves
// headroom for densely-automated mixer states (per-channel volume +
// pan + sends).
constexpr size_t max_param_queues_per_envelope = 32;

// VST3 request-direction envelope.  Carries fixed-shape per-block fields
// that the direct-struct path replaces bitsery for.  Extends per phase:
//   P2 — ProcessContext.
//   P3 — fixed-shape event ring.
//   P4 (this commit) — parameter queue array (input_parameter_changes_).
//
// Layout: cacheline-aligned header (flags + counts), then
// cacheline-aligned payload sections.  All access happens under the L2
// request pi_mutex, so no atomic primitives are needed on internal
// fields.
struct alignas(64) Vst3ProcessEnvelope {
    // Header — first cacheline.
    //   bit 0: process_context_valid
    //   bit 1: input_events_valid (event_count entries from events[])
    //   bit 2: input_param_changes_valid
    //                          (queue_count entries from param_queues[])
    uint32_t flags;
    uint32_t event_count;                    // P3: count of valid events
    uint32_t queue_count;                    // P4: count of valid queues
    uint32_t _hdr_pad[13];                   // remainder of cacheline (52B)

    // ProcessContext payload — own cacheline (offset 64).
    ProcessContextDirect process_context;    // 112 bytes (64..175)

    // Event ring — cacheline-aligned (compiler inserts pad 176..191).
    alignas(64) ProcessEventDirect events[max_events_per_envelope];

    // Parameter queue array — 16-byte aligned, no extra cacheline pad
    // (events[] end at offset 12480, already 16-aligned).  Producer
    // and consumer access serially under the request pi_mutex, so
    // false-sharing concerns inside the array don't apply.
    alignas(16)
        ProcessParamQueueDirect param_queues[max_param_queues_per_envelope];
};
static_assert(sizeof(Vst3ProcessEnvelope) % 64 == 0,
              "Vst3ProcessEnvelope must be a multiple of cacheline size");
static_assert(alignof(Vst3ProcessEnvelope) == 64,
              "Vst3ProcessEnvelope cacheline-aligned");
static_assert(offsetof(Vst3ProcessEnvelope, process_context) == 64,
              "process_context must start at offset 64 (own cacheline)");
static_assert(offsetof(Vst3ProcessEnvelope, events) % 64 == 0,
              "events array must start cacheline-aligned");
static_assert(offsetof(Vst3ProcessEnvelope, events) == 192,
              "events array at offset 192 (cacheline after process_context)");
static_assert(offsetof(Vst3ProcessEnvelope, param_queues) % 16 == 0,
              "param_queues array must start 16-aligned");

// VST3 reply-direction envelope.  Mirror-shape of the request
// envelope's event ring + param queue array, used for output_events_
// and output_parameter_changes_ on the response side (P5).  No
// ProcessContext on the response — that's a request-only field.
//
//   bit 0: output_events_valid
//   bit 1: output_param_changes_valid
//
// Same caps as the request side (256 events, 32 queues × 64 points).
// Layout: header on cacheline 0, events[] on cacheline-aligned
// payload, param_queues[] 16-aligned after events.  Total size
// matches the request envelope minus the 112-byte process_context
// cacheline.
struct alignas(64) Vst3ProcessReplyEnvelope {
    uint32_t flags;
    uint32_t event_count;
    uint32_t queue_count;
    uint32_t _hdr_pad[13];

    alignas(64) ProcessEventDirect events[max_events_per_envelope];
    alignas(16)
        ProcessParamQueueDirect param_queues[max_param_queues_per_envelope];
};
static_assert(sizeof(Vst3ProcessReplyEnvelope) % 64 == 0,
              "Vst3ProcessReplyEnvelope must be a multiple of cacheline size");
static_assert(alignof(Vst3ProcessReplyEnvelope) == 64,
              "Vst3ProcessReplyEnvelope cacheline-aligned");
static_assert(offsetof(Vst3ProcessReplyEnvelope, events) % 64 == 0,
              "reply events must start cacheline-aligned");
static_assert(offsetof(Vst3ProcessReplyEnvelope, param_queues) % 16 == 0,
              "reply param_queues must start 16-aligned");

// Direct-struct representation of VST2's `VstTimeInfo` (vestige
// `aeffectx.h` class).  Field-for-field mirror with explicit-width
// primitives — POD safe across producer (Linux native gcc) and
// consumer (winegcc PE side) ABIs.  88 bytes, 8-byte natural
// alignment.  Conversion helpers in
// `common/serialization/vst2-direct-envelope.h`.
struct VstTimeInfoDirect {
    double  sample_pos;             // 0
    double  sample_rate;             // 8
    double  nano_seconds;            // 16
    double  ppq_pos;                 // 24
    double  tempo;                   // 32
    double  bar_start_pos;           // 40
    double  cycle_start_pos;         // 48
    double  cycle_end_pos;           // 56
    int32_t time_sig_numerator;      // 64
    int32_t time_sig_denominator;    // 68
    uint8_t empty3[12];              // 72  mirrors vestige VstTimeInfo::empty3
    int32_t flags;                   // 84
};
static_assert(sizeof(VstTimeInfoDirect) == 88,
              "VstTimeInfoDirect ABI: 88-byte layout sanity check");
static_assert(alignof(VstTimeInfoDirect) == 8,
              "VstTimeInfoDirect ABI: 8-byte natural alignment");

// VST2 request-direction envelope.  Carries the fixed-shape per-block
// fields that the direct-struct path replaces bitsery for.  P2 (this
// commit) covers VstTimeInfo only — the rest of the small
// `Vst2ProcessRequest` (sample_frames + double_precision +
// current_process_level, ~9 bytes total) stays on the bitsery path
// since the encoding cost is already memcpy-equivalent at that size.
struct alignas(64) Vst2ProcessEnvelope {
    // Header — first cacheline.  bit 0: time_info_valid.
    uint32_t flags;
    uint32_t _hdr_pad[15];           // remainder of cacheline

    // VstTimeInfo payload — starts at offset 64 (next cacheline).
    VstTimeInfoDirect time_info;     // 88 bytes
    // trailing pad to next 64-byte boundary handled by struct alignas
};
static_assert(sizeof(Vst2ProcessEnvelope) % 64 == 0,
              "Vst2ProcessEnvelope must be a multiple of cacheline size");
static_assert(alignof(Vst2ProcessEnvelope) == 64,
              "Vst2ProcessEnvelope cacheline-aligned");
static_assert(offsetof(Vst2ProcessEnvelope, time_info) == 64,
              "time_info must start at offset 64 (own cacheline)");

// Direct-struct mirror of `clap_event_header_t`.  16-byte POD,
// 4-byte alignment.  Field-for-field match with the CLAP SDK header.
struct ClapEventHeaderDirect {
    uint32_t size;
    uint32_t time;
    uint16_t space_id;
    uint16_t type;
    uint32_t flags;
};
static_assert(sizeof(ClapEventHeaderDirect) == 16,
              "ClapEventHeaderDirect ABI: 16-byte layout sanity check");
static_assert(alignof(ClapEventHeaderDirect) == 4,
              "ClapEventHeaderDirect ABI: 4-byte natural alignment");

// Direct-struct mirror of fixed-shape CLAP events (Note / NoteExpression
// / ParamValue / ParamMod / ParamGesture / Midi / Midi2).  Variable-
// shape variants (Transport-as-event and MidiSysex) fall back to the
// bitsery path for that block.  64 bytes total — 16-byte header +
// 48-byte payload union sized to the largest fixed variant
// (ParamValue / ParamMod: 40 bytes used + 8 pad).  Cacheline-friendly
// (sizeof multiple of 16; 4 events per cacheline).  Conversion helpers
// in `common/serialization/clap-direct-envelope.h`.
struct alignas(16) ClapProcessEventDirect {
    ClapEventHeaderDirect header;        // 0  (16 bytes)
    union {
        // clap_event_note_t — NOTE_ON / NOTE_OFF / NOTE_CHOKE / NOTE_END
        struct {
            int32_t  note_id;            // 0
            int16_t  port_index;         // 4
            int16_t  channel;            // 6
            int16_t  key;                // 8
            int16_t  _pad0;              // 10
            uint32_t _pad1;              // 12
            double   velocity;           // 16
            uint8_t  _tail_pad[24];      // 24
        } note;                          // 48 bytes
        // clap_event_note_expression_t
        struct {
            int32_t  expression_id;      // 0
            int32_t  note_id;            // 4
            int16_t  port_index;         // 8
            int16_t  channel;            // 10
            int16_t  key;                // 12
            int16_t  _pad0;              // 14
            double   value;              // 16
            uint8_t  _tail_pad[24];      // 24
        } note_expression;               // 48 bytes
        // clap_event_param_value_t / clap_event_param_mod_t — same shape
        struct {
            uint32_t param_id;           // 0
            uint32_t _pad0;              // 4
            uint64_t cookie;             // 8   void* opaque
            int32_t  note_id;            // 16
            int16_t  port_index;         // 20
            int16_t  channel;            // 22
            int16_t  key;                // 24
            int16_t  _pad1;              // 26
            uint32_t _pad2;              // 28
            double   value_or_amount;    // 32
            uint8_t  _tail_pad[8];       // 40
        } param_value_or_mod;            // 48 bytes
        // clap_event_param_gesture_t — BEGIN / END
        struct {
            uint32_t param_id;           // 0
            uint8_t  _tail_pad[44];      // 4
        } param_gesture;                 // 48 bytes
        // clap_event_midi_t
        struct {
            uint16_t port_index;         // 0
            uint8_t  data[3];            // 2
            uint8_t  _tail_pad[43];      // 5
        } midi;                          // 48 bytes
        // clap_event_midi2_t
        struct {
            uint16_t port_index;         // 0
            uint16_t _pad0;              // 2
            uint32_t data[4];            // 4
            uint8_t  _tail_pad[28];      // 20
        } midi2;                         // 48 bytes
        uint8_t raw[48];
    } payload;                           // 48 bytes — starts at offset 16
};
static_assert(sizeof(ClapProcessEventDirect) == 64,
              "ClapProcessEventDirect ABI: 64-byte layout sanity check");
static_assert(alignof(ClapProcessEventDirect) == 16,
              "ClapProcessEventDirect ABI: 16-byte natural alignment");
static_assert(offsetof(ClapProcessEventDirect, payload) == 16,
              "ClapProcessEventDirect payload at offset 16");

// Maximum number of fixed-shape CLAP events the envelope can carry in
// one block.  Sized for typical synthesizer load (one event per few
// samples) at 256-sample buffer sizes.  Variable-shape variants
// (Transport-as-event, MidiSysex) or counts above this force the
// entire input event list back onto the bitsery path for that block.
constexpr size_t max_clap_events_per_envelope = 256;

// Direct-struct mirror of `clap_event_transport_t`.  All-POD wire
// format with explicit padding to keep 8-byte int64/double natural
// alignment.  112 bytes total.  Conversion helpers in
// `common/serialization/clap-direct-envelope.h`.
struct ClapTransportDirect {
    ClapEventHeaderDirect header;       // 0   (16 bytes)
    uint32_t flags;                     // 16  clap_transport_flags
    uint32_t _pad0;                     // 20  align next int64 to 24
    int64_t  song_pos_beats;            // 24
    int64_t  song_pos_seconds;          // 32
    double   tempo;                     // 40
    double   tempo_inc;                 // 48
    int64_t  loop_start_beats;          // 56
    int64_t  loop_end_beats;            // 64
    int64_t  loop_start_seconds;        // 72
    int64_t  loop_end_seconds;          // 80
    int64_t  bar_start;                 // 88
    int32_t  bar_number;                // 96
    uint16_t tsig_num;                  // 100
    uint16_t tsig_denom;                // 102
    uint32_t _trail_pad;                // 104  align size to multiple of 8
};
static_assert(sizeof(ClapTransportDirect) == 112,
              "ClapTransportDirect ABI: 112-byte layout sanity check");
static_assert(alignof(ClapTransportDirect) == 8,
              "ClapTransportDirect ABI: 8-byte natural alignment");

// CLAP request-direction envelope.  Carries fixed-shape per-block
// fields that the direct-struct path replaces bitsery for.
//   P2 — clap_event_transport_t (transport_).
//   P3 (this commit) — fixed-shape event ring (in_events_).
//                      Variable-shape variants (Transport-as-event,
//                      MidiSysex) keep the entire block on bitsery.
struct alignas(64) ClapProcessEnvelope {
    // Header — first cacheline.
    //   bit 0: transport_valid
    //   bit 1: in_events_valid (event_count entries in events[])
    uint32_t flags;
    uint32_t event_count;                // P3: count of valid events
    uint32_t _hdr_pad[14];

    // Transport payload — own cacheline (offset 64).
    ClapTransportDirect transport;       // 112 bytes (64..175)

    // Event ring — cacheline-aligned (compiler inserts pad 176..191).
    alignas(64) ClapProcessEventDirect events[max_clap_events_per_envelope];
};
static_assert(sizeof(ClapProcessEnvelope) % 64 == 0,
              "ClapProcessEnvelope must be a multiple of cacheline size");
static_assert(alignof(ClapProcessEnvelope) == 64,
              "ClapProcessEnvelope cacheline-aligned");
static_assert(offsetof(ClapProcessEnvelope, transport) == 64,
              "transport must start at offset 64 (own cacheline)");
static_assert(offsetof(ClapProcessEnvelope, events) % 64 == 0,
              "events array must start cacheline-aligned");
static_assert(offsetof(ClapProcessEnvelope, events) == 192,
              "events array at offset 192 (cacheline after transport)");

// CLAP reply-direction envelope (P5).  Carries out_events_ as a fixed-
// shape event ring; no transport (transport is request-only).  Same
// caps and ABI rules as the request side; variable-shape variants
// (MidiSysex / Transport-as-event) force the whole output stream onto
// the bitsery path for that block.
//
//   bit 0: output_events_valid
struct alignas(64) ClapProcessReplyEnvelope {
    uint32_t flags;
    uint32_t event_count;
    uint32_t _hdr_pad[14];

    alignas(64) ClapProcessEventDirect events[max_clap_events_per_envelope];
};
static_assert(sizeof(ClapProcessReplyEnvelope) % 64 == 0,
              "ClapProcessReplyEnvelope must be a multiple of cacheline size");
static_assert(alignof(ClapProcessReplyEnvelope) == 64,
              "ClapProcessReplyEnvelope cacheline-aligned");
static_assert(offsetof(ClapProcessReplyEnvelope, events) % 64 == 0,
              "CLAP reply events must start cacheline-aligned");

// Environment variable that opts OUT of the direct-struct envelope path.
// Default ON — verified 2026-05-13 on representative workload (ACE VST3
// multi-core, dense MIDI CC + automation): bitsery encode/decode dropped
// from a measurable hot symbol to 0.10% of wine-host CPU combined,
// pi_mutex_lock to 0.01%.  See commit message for full findings.
//
// YABRIDGE_DIRECT_ENVELOPE=0 forces every block onto the bitsery path —
// kept as a diagnostic opt-out for A/B comparison and bug triage.  Any
// other value (including unset) keeps the envelope path on.
constexpr char direct_envelope_env_var[] = "YABRIDGE_DIRECT_ENVELOPE";

// Returns true unless YABRIDGE_DIRECT_ENVELOPE=0 in the environment.
// Cached on first call — subsequent calls do not touch the environment,
// so this is safe to call from RT contexts after process startup.  The
// initial call is NOT RT-safe (it does getenv), so call sites should
// trigger it before the audio thread starts.
bool direct_envelope_enabled() noexcept;

// Header layout — sits at offset 0 of the shmem region. Two cache-line-
// aligned synchronization PAIRS (one per direction), then state +
// metadata on a separate line, then the fixed-size request/reply
// buffers.
//
// Cache-line alignment for pi_mutex/pi_cond is critical: FUTEX_LOCK_PI's
// kernel waiter hash table buckets by address modulo cache line.
// Crossing a cache line can confuse the kernel's PI chain tracking
// under contention.
struct AudioControlShmLayout {
    // Request direction — producer takes this to publish a request,
    // consumer takes this to receive.
    alignas(64) pi_mutex_t req_lock;
    alignas(64) pi_cond_t  req_cv;

    // Reply direction — consumer takes this to publish a reply,
    // producer takes this to receive.
    alignas(64) pi_mutex_t reply_lock;
    alignas(64) pi_cond_t  reply_cv;

    // State + metadata. State is atomically loaded under either lock.
    // Writes to state happen under the lock for the direction that's
    // publishing the new state value.
    alignas(64) std::atomic<uint32_t> state;
    uint32_t request_size;
    uint32_t reply_size;
    uint32_t generation;
    uint8_t  _state_pad[64 - 16];

    alignas(64) uint8_t request_buf[audio_control_buf_size];
    alignas(64) uint8_t reply_buf[audio_control_buf_size];

    // === NSPA L2 direct-struct envelope extension (commit 2+) ===
    //
    // Layout version + use-direct flag, cacheline-aligned header.  Both
    // are written by the creator at Create time and read by the peer
    // at Attach time.  Once Attach has completed, neither field is
    // modified for the lifetime of the region.
    //
    // envelope_layout_version: creator writes audio_control_layout_version.
    // Peer compares to its own compile-time value; mismatch → disable
    // direct path, fall back to bitsery transparently.
    //
    // envelope_use_direct: 1 iff creator had YABRIDGE_DIRECT_ENVELOPE=1
    // AND version match observed.  Both sides check this before
    // writing/reading envelope payload fields.
    alignas(64) std::atomic<uint32_t> envelope_layout_version;
    uint32_t envelope_use_direct;
    uint8_t _env_hdr_pad[64 - 8];

    // Per-format request envelope sections.  Only the section for this
    // instance's plugin format is populated; others stay zeroed.
    alignas(64) Vst3ProcessEnvelope request_envelope_vst3;
    alignas(64) Vst2ProcessEnvelope request_envelope_vst2;
    alignas(64) ClapProcessEnvelope request_envelope_clap;

    // Per-format reply envelope sections.  VST3 mirrors both events
    // and param queues (P5).  CLAP mirrors out_events_.  VST2 reply
    // is `Ack` (zero-byte response — audio data is in the
    // AudioShmBuffer, no output event/param surface), so no VST2
    // reply envelope exists.
    alignas(64) Vst3ProcessReplyEnvelope reply_envelope_vst3;
    alignas(64) ClapProcessReplyEnvelope reply_envelope_clap;
};

static_assert(offsetof(AudioControlShmLayout, req_lock) == 0,
              "req_lock must be at offset 0 for ABI stability");
static_assert(sizeof(AudioControlShmLayout) >=
                  (2 * audio_control_buf_size + 320 + 64 +
                   sizeof(Vst3ProcessEnvelope) +
                   sizeof(Vst2ProcessEnvelope) +
                   sizeof(ClapProcessEnvelope) +
                   sizeof(Vst3ProcessReplyEnvelope) +
                   sizeof(ClapProcessReplyEnvelope)),
              "layout sanity check (with all envelope extensions)");

// RAII handle to an AudioControlShm region. Holds the shm_open fd and
// the mmap pointer. Constructor mode controls whether we create
// (creator) or attach (peer).
class AudioControlShm {
   public:
    struct Create {};
    struct Attach {};

    // Creator constructor — plugin-lib side. Allocates a fresh shmem
    // region, sizes it, mmaps it, initializes both pi_mutex+pi_cond
    // pairs with PSHARED flags. Throws on any failure.
    //
    // `name` must be a leading-slash path suitable for shm_open
    // (e.g. "/yabridge-audio-ctl-<plugin>-<random>").
    AudioControlShm(Create, const std::string& name);

    // Peer constructor — wine-host side. Opens an already-created
    // shmem region by name, mmaps it. Does NOT re-initialize the
    // sync primitives. Throws on any failure.
    AudioControlShm(Attach, const std::string& name);

    ~AudioControlShm() noexcept;

    AudioControlShm(const AudioControlShm&) = delete;
    AudioControlShm& operator=(const AudioControlShm&) = delete;
    AudioControlShm(AudioControlShm&& o) noexcept;
    AudioControlShm& operator=(AudioControlShm&&) = delete;

    AudioControlShmLayout& layout() noexcept { return *layout_; }
    const AudioControlShmLayout& layout() const noexcept { return *layout_; }

    pi_mutex_t* req_lock()   noexcept { return &layout_->req_lock; }
    pi_cond_t*  req_cv()     noexcept { return &layout_->req_cv; }
    pi_mutex_t* reply_lock() noexcept { return &layout_->reply_lock; }
    pi_cond_t*  reply_cv()   noexcept { return &layout_->reply_cv; }

    const std::string& name() const noexcept { return name_; }
    bool is_creator() const noexcept { return is_creator_; }

    // Whether the direct-struct envelope path is active for THIS side
    // of THIS region.  Set at construction time:
    //   - Creator: true iff YABRIDGE_DIRECT_ENVELOPE=1 in env.
    //   - Peer: true iff creator's envelope_layout_version matches the
    //     peer's compile-time `audio_control_layout_version` AND the
    //     creator set envelope_use_direct=1.
    //
    // Once set, this flag is stable for the lifetime of the region.
    // RT-safe to query (plain bool, no atomic, no syscalls).  Call
    // sites consult this per audio block to decide whether to read/
    // write the envelope payload fields.  If false, the existing
    // bitsery+shmem path is used unchanged.
    bool envelope_active() const noexcept { return envelope_active_; }

    // Signal both sides to exit their loops. Atomically sets
    // state = Shutdown and broadcasts both conds so any waiter
    // (typically the wine-host audio thread blocked in pi_cond_wait
    // on either direction) wakes and observes the state. Safe to
    // call from either side; idempotent.
    //
    // Must be called BEFORE this object's destructor runs (or at least
    // before the shmem is unmapped by either process). Called from the
    // parent bridge's explicit destructor before its Win32Thread
    // members destruct, so the threads observe the signal and exit
    // before being joined.
    void signal_shutdown() noexcept;

   private:
    std::string name_;
    int fd_ = -1;
    AudioControlShmLayout* layout_ = nullptr;
    bool is_creator_ = false;
    bool is_moved_ = false;
    bool envelope_active_ = false;
};

// =====================================================================
// Hot-path helpers — used at audio call sites.
//
// `request_bytes` is copied INTO the shmem request slot.
// `reply_bytes_out` receives the reply (caller-provided buffer).
//
// Both functions throw std::overflow_error if payload sizes exceed the
// fixed slot size. Callers fall back to socket transport on overflow.
//
// Other failures (pi_mutex_lock errno, pi_cond_wait errno) throw
// std::system_error.
//
// NB: these helpers do NOT allocate. The buffer pointers must reference
// memory whose lifetime spans the call.
// =====================================================================

// Producer side — DAW's audio thread (plugin-lib).
// Lock-span: only across state transitions + memcpy. Plugin processing
// happens between the two unlock points on the consumer side, so the
// producer's wait on reply_cv is bounded by plugin time only.
void audio_control_send_and_wait(AudioControlShm& shm,
                                 const uint8_t* request_bytes,
                                 size_t request_size,
                                 uint8_t* reply_bytes_out,
                                 size_t reply_bytes_capacity,
                                 size_t& reply_size_out);

// Consumer side — wine-host audio worker thread.
// `handler` is invoked WITHOUT any cross-process lock held. Handler
// receives the request bytes (already copied to caller-provided
// `request_local`) and a writable reply buffer (caller-provided
// `reply_local`). Handler returns the reply byte count.
//
// `Handler` signature:
//   size_t(const uint8_t* request, size_t request_size,
//          uint8_t* reply_out, size_t reply_capacity)
//
// Throws AudioControlShutdown when the Shutdown state is observed.
template <typename Handler>
void audio_control_recv_one(AudioControlShm& shm,
                            uint8_t* request_local,
                            size_t request_local_capacity,
                            uint8_t* reply_local,
                            size_t reply_local_capacity,
                            Handler&& handler) {
    pi_mutex_t* req_mu = shm.req_lock();
    pi_cond_t*  req_cv = shm.req_cv();
    auto& layout = shm.layout();

    // === Request direction ===
    if (pi_mutex_lock(req_mu) != 0) {
        throw std::system_error(errno, std::system_category(),
                                "pi_mutex_lock(req) in recv_one");
    }

    size_t request_size = 0;
    for (;;) {
        const uint32_t s = layout.state.load(std::memory_order_acquire);
        if (s == static_cast<uint32_t>(AudioControlState::RequestReady)) {
            // Snapshot under lock; copy to caller-owned buffer.
            request_size = layout.request_size;
            if (request_size > request_local_capacity) {
                pi_mutex_unlock(req_mu);
                throw std::overflow_error(
                    "audio_control_recv_one: request larger than local buffer");
            }
            std::memcpy(request_local, layout.request_buf, request_size);
            break;
        }
        if (s == static_cast<uint32_t>(AudioControlState::Shutdown)) {
            pi_mutex_unlock(req_mu);
            throw AudioControlShutdown{};
        }
        if (pi_cond_wait(req_cv, req_mu) != 0) {
            pi_mutex_unlock(req_mu);
            throw std::system_error(errno, std::system_category(),
                                    "pi_cond_wait(req) in recv_one");
        }
    }

    pi_mutex_unlock(req_mu);
    // ===== NO cross-process lock held below this point =====

    const size_t reply_size = handler(request_local, request_size,
                                      reply_local, reply_local_capacity);

    if (reply_size > audio_control_buf_size) {
        // Shouldn't happen — handler should respect reply_local_capacity
        // which we passed in. Defence-in-depth: throw rather than
        // corrupt the shmem reply slot.
        throw std::overflow_error(
            "audio_control_recv_one: handler returned oversized reply");
    }

    // === Reply direction ===
    pi_mutex_t* reply_mu = shm.reply_lock();
    pi_cond_t*  reply_cv = shm.reply_cv();
    if (pi_mutex_lock(reply_mu) != 0) {
        throw std::system_error(errno, std::system_category(),
                                "pi_mutex_lock(reply) in recv_one");
    }

    std::memcpy(layout.reply_buf, reply_local, reply_size);
    layout.reply_size = static_cast<uint32_t>(reply_size);
    layout.state.store(static_cast<uint32_t>(AudioControlState::ReplyReady),
                       std::memory_order_release);

    if (pi_cond_signal(reply_cv, reply_mu) != 0) {
        pi_mutex_unlock(reply_mu);
        throw std::system_error(errno, std::system_category(),
                                "pi_cond_signal(reply) in recv_one");
    }

    pi_mutex_unlock(reply_mu);
}

}  // namespace yabridge::nspa
