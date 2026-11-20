#include "pm_api/param_table.h"

#include <cstring>

namespace {

constexpr int A = PM_PARAM_CLASS_REALTIME;
constexpr int B = PM_PARAM_CLASS_PREPARE;
constexpr int AUTO = PM_PARAM_FLAG_AUTOMATABLE;
constexpr int HID = PM_PARAM_FLAG_HIDDEN;
constexpr int SM = PM_PARAM_FLAG_SMOOTHED;
constexpr int REJ = PM_PARAM_FLAG_REJECTS_OUT_OF_RANGE;
constexpr int NRT = PM_PARAM_FLAG_NON_REALTIME_SET;
constexpr int LIN = PM_PARAM_CURVE_LINEAR;
constexpr int LOG = PM_PARAM_CURVE_LOG;
constexpr int DB = PM_PARAM_CURVE_DB;
constexpr int BOOL = PM_PARAM_CURVE_BOOLEAN;
constexpr int INT = PM_PARAM_CURVE_INTEGER;

constexpr const char* kSchedulingNames[] = {"lowest_latency", "constant_latency"};
constexpr const char* kTierNames[] = {"full", "balanced", "eco"};
constexpr const char* kTouchNames[] = {"light", "medium", "strong"};

// Keys are stable forever (pm_api.h); never rename one. Ranges: where the
// engine clamped or rejected before API 0.17 the same bounds are used; the
// others were unbounded at set_param and are bounded here at or beyond the
// bounds the DSP applies, so every value that was valid behaves as before.
// Defaults mirror the engine's initial values (engine.h, piano_table.h,
// SoundboardConfig, Room) and the pm_host_create overrides.
constexpr pm_param_info_t kParams[] = {
    // id, key, name, unit, min, max, default (product), engine default, curve, class, flags, enum
    {PM_PARAM_MASTER_GAIN_DB, "master_gain_db", "Master Volume", "dB", -96.0, 24.0, 14.0, 0.0, DB, A, AUTO | SM, 0, nullptr},
    {PM_PARAM_STEREO_WIDTH, "stereo_width", "Stereo Width", "", 0.0, 1.0, 0.7, 0.7, LIN, A, AUTO, 0, nullptr},
    {PM_PARAM_SCHEDULING_MODE, "scheduling_mode", "Scheduling Mode", "", 0.0, 1.0, 0.0, 0.0, INT, A, HID, 2, kSchedulingNames},
    // 0 = max_frames of the next prepare (prepare stores that value).
    {PM_PARAM_CONSTANT_LATENCY_FRAMES, "constant_latency_frames", "Constant Latency", "frames", 0.0, 8192.0, 0.0, 0.0, INT, A, HID, 0, nullptr},
    {PM_PARAM_CULL_THRESHOLD_DB, "cull_threshold_db", "Culling Threshold", "dB", -200.0, 0.0, -100.0, -100.0, DB, A, HID, 0, nullptr},
    {PM_PARAM_CULLING_ENABLED, "culling_enabled", "Culling", "", 0.0, 1.0, 1.0, 1.0, BOOL, A, HID, 0, nullptr},
    {PM_PARAM_VELOCITY_MIN, "velocity_min_mps", "Softest Hammer Speed", "m/s", 0.01, 10.0, 0.3, 0.3, LOG, A, HID | NRT, 0, nullptr},
    {PM_PARAM_VELOCITY_MAX, "velocity_max_mps", "Loudest Hammer Speed", "m/s", 0.01, 10.0, 5.5, 5.5, LOG, A, HID | NRT, 0, nullptr},
    {PM_PARAM_VELOCITY_EXPONENT, "velocity_exponent", "Velocity Curve", "", 0.1, 5.0, 1.5, 1.5, LOG, A, HID | NRT, 0, nullptr},
    {PM_PARAM_A4_HZ, "a4_hz", "Reference Pitch A4", "Hz", 300.0, 600.0, 440.0, 440.0, LIN, B, REJ | NRT, 0, nullptr},
    {PM_PARAM_TUNING_STRETCH, "tuning_stretch", "Stretch Tuning", "", 0.0, 1.5, 1.0, 1.0, LIN, B, NRT, 0, nullptr},
    {PM_PARAM_EQUAL_TEMPERAMENT, "equal_temperament", "Plain Equal Temperament", "", 0.0, 1.0, 0.0, 0.0, BOOL, B, NRT, 0, nullptr},
    {PM_PARAM_UNISON_WIDTH, "unison_width", "Unison Width", "", 0.0, 5.0, 1.0, 1.0, LIN, B, NRT, 0, nullptr},
    {PM_PARAM_SOUNDBOARD_ENABLED, "soundboard_enabled", "Soundboard", "", 0.0, 1.0, 1.0, 1.0, BOOL, B, HID, 0, nullptr},
    {PM_PARAM_SYMPATHETIC, "sympathetic", "Sympathetic Resonance", "", 0.0, 1.5, 1.0, 1.0, LIN, B, 0, 0, nullptr},
    {PM_PARAM_KNOCK_LEVEL, "knock_level", "Hammer Knock", "N/(m/s)", 0.0, 1.0, 0.02, 0.02, LIN, A, HID, 0, nullptr},
    {PM_PARAM_DAMPER_NOISE_LEVEL, "damper_noise_level", "Damper Noise", "N", 0.0, 0.1, 0.004, 0.004, LIN, A, HID, 0, nullptr},
    {PM_PARAM_HALF_PEDAL_P0_TREBLE, "half_pedal_p0_treble", "Half Pedal Start (Treble)", "", 0.0, 1.0, 0.30, 0.30, LIN, A, REJ, 0, nullptr},
    {PM_PARAM_HALF_PEDAL_P0_BASS, "half_pedal_p0_bass", "Half Pedal Start (Bass)", "", 0.0, 1.0, 0.40, 0.40, LIN, A, REJ, 0, nullptr},
    // The engine stores at least 0.01 (get_param returns 0.01 after 0).
    {PM_PARAM_HALF_PEDAL_WIDTH, "half_pedal_width", "Half Pedal Width", "", 0.0, 1.0, 0.25, 0.25, LIN, A, REJ, 0, nullptr},
    {PM_PARAM_DAMPED_CULL_OFFSET_DB, "damped_cull_offset_db", "Damped Wake Offset", "dB", 0.0, 200.0, 120.0, 120.0, DB, A, HID, 0, nullptr},
    {PM_PARAM_QUALITY_TIER, "quality_tier", "Quality", "", 0.0, 2.0, 0.0, 0.0, INT, B, REJ, 3, kTierNames},
    {PM_PARAM_LOOP_CUTOFF_HZ, "loop_cutoff_hz", "Loop Band Override", "Hz", 0.0, 20000.0, 0.0, 0.0, LIN, B, HID, 0, nullptr},
    {PM_PARAM_ROOM_ENABLED, "room_enabled", "Room", "", 0.0, 1.0, 1.0, 0.0, BOOL, A, AUTO | SM, 0, nullptr},
    {PM_PARAM_ROOM_LEVEL_DB, "room_level_db", "Room Level", "dB", -60.0, 12.0, -9.0, -9.0, DB, A, AUTO | SM, 0, nullptr},
    {PM_PARAM_ROOM_DECAY, "room_decay_s", "Room Decay", "s", 0.3, 6.0, 1.8, 1.8, LOG, A, AUTO | SM, 0, nullptr},
    {PM_PARAM_ROOM_HF_RATIO, "room_hf_ratio", "Room HF Decay Ratio", "", 0.1, 1.0, 0.5, 0.5, LIN, A, AUTO | SM, 0, nullptr},
    {PM_PARAM_ROOM_SIZE, "room_size", "Room Size", "", 0.5, 2.0, 1.0, 1.0, LOG, A, AUTO | SM, 0, nullptr},
    {PM_PARAM_ROOM_EARLY_LATE, "room_early_late", "Early / Late Balance", "", 0.0, 1.0, 0.5, 0.5, LIN, A, AUTO | SM, 0, nullptr},
    {PM_PARAM_TREBLE_DB, "treble_db", "Treble", "dB", -12.0, 6.0, -9.0, -9.0, DB, A, AUTO | SM, 0, nullptr},
    {PM_PARAM_VOICING_FELT_HARDNESS, "voicing_felt_hardness", "Felt Hardness", "", 0.25, 4.0, 1.0, 1.0, LOG, B, HID | NRT, 0, nullptr},
    {PM_PARAM_VOICING_FELT_EXPONENT, "voicing_felt_exponent", "Felt Stiffening Offset", "", -1.0, 1.0, 0.0, 0.0, LIN, B, HID | NRT, 0, nullptr},
    {PM_PARAM_VOICING_HAMMER_MASS, "voicing_hammer_mass", "Hammer Mass", "", 0.5, 2.0, 1.0, 1.0, LOG, B, HID | NRT, 0, nullptr},
    {PM_PARAM_VOICING_STRIKE_POSITION, "voicing_strike_position", "Strike Position", "", 0.5, 2.0, 1.0, 1.0, LOG, B, HID | NRT, 0, nullptr},
    {PM_PARAM_VOICING_STRING_LOSS, "voicing_string_loss", "String Loss", "", 0.25, 4.0, 1.0, 1.0, LOG, B, HID | NRT, 0, nullptr},
    {PM_PARAM_VOICING_HF_LOSS, "voicing_hf_loss", "String HF Loss", "", 0.25, 4.0, 1.0, 1.0, LOG, B, HID | NRT, 0, nullptr},
    {PM_PARAM_VOICING_INHARMONICITY, "voicing_inharmonicity", "Inharmonicity", "", 0.0, 2.0, 1.0, 1.0, LIN, B, HID | NRT, 0, nullptr},
    {PM_PARAM_VOICING_BOARD_DAMPING, "voicing_board_damping", "Board Damping", "", 0.25, 4.0, 1.0, 1.0, LOG, B, HID | NRT, 0, nullptr},
    {PM_PARAM_TOUCH, "touch", "Touch", "", 0.0, 2.0, 1.0, 1.0, INT, A, NRT, 3, kTouchNames},
    {PM_PARAM_VOICING_RADIATION_TILT, "voicing_radiation_tilt", "Radiation Tilt", "dB/oct", -9.0, 0.0, 0.0, 0.0, LIN, B, HID | NRT, 0, nullptr},
    {PM_PARAM_VOICING_RADIATION_CORNER_HZ, "voicing_radiation_corner_hz", "Radiation Tilt Corner", "Hz", 250.0, 8000.0, 1000.0, 1000.0, LOG, B, HID | NRT, 0, nullptr},
    {PM_PARAM_VOICING_FELT_DAMPING, "voicing_felt_damping", "Felt Damping", "", 0.25, 8.0, 1.0, 1.0, LOG, B, HID | NRT, 0, nullptr},
};

constexpr int kCount = static_cast<int>(sizeof(kParams) / sizeof(kParams[0]));

// The table is indexed by id: ids are dense from 0.
constexpr bool idsDense() {
    for (int i = 0; i < kCount; ++i)
        if (kParams[i].id != i) return false;
    return true;
}
static_assert(idsDense(), "kParams must list ids 0..N-1 in order");
static_assert(kCount == PM_PARAM_VOICING_FELT_DAMPING + 1, "every pm_param needs a descriptor");

}  // namespace

namespace pm {

const pm_param_info_t* paramDescriptor(std::int32_t id) noexcept {
    return id >= 0 && id < kCount ? &kParams[id] : nullptr;
}

bool paramAccept(const pm_param_info_t& d, double* value) noexcept {
    const double v = *value;
    if (v != v) return false;
    if (v >= d.min && v <= d.max) return true;
    if (d.flags & PM_PARAM_FLAG_REJECTS_OUT_OF_RANGE) return false;
    *value = v < d.min ? d.min : d.max;
    return true;
}

}  // namespace pm

extern "C" {

int32_t pm_param_count(void) { return kCount; }

int pm_param_info(int32_t index, pm_param_info_t* out) {
    if (!out || index < 0 || index >= kCount) return 0;
    *out = kParams[index];
    return 1;
}

int32_t pm_param_id_from_key(const char* key) {
    if (!key) return -1;
    for (const pm_param_info_t& d : kParams)
        if (std::strcmp(d.key, key) == 0) return d.id;
    return -1;
}

}  // extern "C"
