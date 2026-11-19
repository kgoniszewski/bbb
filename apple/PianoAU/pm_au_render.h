// Render-side logic of the AUv3 instrument (plan §5.2) as plain C++ over the
// C API, so it compiles and can be checked without Apple frameworks. The
// Objective-C++ render block in PianoAudioUnit.mm walks the AURenderEvent
// list and calls these functions; nothing here allocates, locks or blocks.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "pm_api.h"

namespace pm_au {

enum ParamKind : std::uint8_t {
    kParamUnknown = 0,
    kParamRealtime = 1,        // class A: set from any thread, including the render thread
    kParamNonRealtimeSet = 2,  // class A but PM_PARAM_FLAG_NON_REALTIME_SET (velocity curve)
    kParamPrepare = 3          // class B: takes effect at the next pm_host_prepare
};

constexpr std::int32_t kMaxParamIds = 256;

// Shared between the AU object (non-real-time) and its render block. The
// non-real-time side writes ticksPerSample, maxFrames and the scratch
// buffers only inside allocateRenderResources, when the host guarantees
// that the render block is not running.
struct RenderState {
    pm_host* host = nullptr;
    pm_engine* engine = nullptr;
    double ticksPerSample = 0.0;
    std::int32_t maxFrames = 0;
    float* scratchLeft = nullptr;   // maxFrames each; used when the host gives null mData
    float* scratchRight = nullptr;
    std::uint8_t paramKind[kMaxParamIds] = {};
};

inline void classifyParams(RenderState& s) noexcept {
    for (std::int32_t i = 0; i < kMaxParamIds; ++i) s.paramKind[i] = kParamUnknown;
    const std::int32_t n = pm_param_count();
    for (std::int32_t i = 0; i < n; ++i) {
        pm_param_info_t info;
        if (!pm_param_info(i, &info) || info.id < 0 || info.id >= kMaxParamIds) continue;
        std::uint8_t kind = kParamRealtime;
        if (info.cls == PM_PARAM_CLASS_PREPARE) kind = kParamPrepare;
        else if (info.flags & PM_PARAM_FLAG_NON_REALTIME_SET) kind = kParamNonRealtimeSet;
        s.paramKind[info.id] = kind;
    }
}

inline ParamKind paramKind(const RenderState& s, std::int64_t address) noexcept {
    if (address < 0 || address >= kMaxParamIds) return kParamUnknown;
    return static_cast<ParamKind>(s.paramKind[address]);
}

// Frame of the buffer an event belongs to. AU event times are absolute
// sample times; AUEventSampleTimeImmediate (a large negative value, plus an
// optional offset) and anything already past land at frame 0.
inline std::int32_t frameOffset(std::int64_t eventSampleTime, double bufferSampleTime,
                                std::int32_t frames) noexcept {
    if (frames <= 0) return 0;
    const double d = static_cast<double>(eventSampleTime) - bufferSampleTime;
    if (!(d > 0.0)) return 0;
    if (d >= static_cast<double>(frames - 1)) return frames - 1;
    return static_cast<std::int32_t>(d);
}

// Host time of a frame of the buffer. pm_host maps a host time t to frame
// floor((t - start) / ticksPerSample); rounding up here keeps that floor at
// exactly `frame` (it would land one frame early for some ratios otherwise).
inline std::uint64_t hostTimeAtFrame(std::uint64_t bufferHostTime, std::int32_t frame,
                                     double ticksPerSample) noexcept {
    if (frame <= 0 || !(ticksPerSample > 0.0)) return bufferHostTime;
    return bufferHostTime + static_cast<std::uint64_t>(std::ceil(frame * ticksPerSample));
}

// MIDI 1.0 bytes of one AURenderEventMIDI (complete messages, at most 3 bytes).
inline void midiBytes(const RenderState& s, const std::uint8_t* bytes, std::int32_t count,
                      std::uint64_t hostTime, std::uint32_t cable) noexcept {
    if (count > 0) pm_host_midi_bytes(s.host, bytes, count, hostTime, cable);
}

// The UMP words of one MIDIEventPacket (MIDI 1.0 or 2.0 protocol).
inline void midiUmp(const RenderState& s, const std::uint32_t* words, std::int32_t count,
                    std::uint64_t hostTime, std::uint32_t cable) noexcept {
    if (count > 0) pm_host_midi_ump(s.host, words, count, hostTime, cable);
}

// AURenderEventParameter / ParameterRamp. Only class A parameters that may be
// set from the audio thread are applied; the engine smooths the SMOOTHED
// ones, so a ramp goes straight to its target. Returns 1 if applied.
inline int parameterEvent(const RenderState& s, std::int64_t address, float value) noexcept {
    if (paramKind(s, address) != kParamRealtime) return 0;
    return pm_engine_set_param(s.engine, static_cast<std::int32_t>(address), static_cast<double>(value));
}

// One pm_host_render for the whole buffer (the AU guarantees frames <=
// maximumFramesToRender, which is what the host was prepared with). right
// may equal nullptr for a mono bus: the right channel then goes to scratch.
inline void render(const RenderState& s, float* left, float* right, std::int32_t frames,
                   std::uint64_t hostTime) noexcept {
    if (frames <= 0) return;
    if (!left) left = s.scratchLeft;
    if (!right) right = s.scratchRight;
    if (!left || !right) return;
    pm_host_render(s.host, left, right, frames, hostTime);
}

}  // namespace pm_au
