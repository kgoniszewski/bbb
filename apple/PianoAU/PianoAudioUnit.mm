#import "PianoAudioUnit.h"

#import <CoreMIDI/CoreMIDI.h>
#include <mach/mach_time.h>

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <new>

#include "pm_api.h"
#include "pm_au_render.h"

NSString* const PianoAudioUnitPresetKey = @"pmPreset";

namespace {

// Product defaults for the non-hidden parameters (factory preset 0).
constexpr char kDefaultPresetJSON[] = "{\"format\":\"pm-preset\",\"version\":1,\"name\":\"Default\",\"params\":{}}";

AudioUnitParameterUnit unitFor(const pm_param_info_t& info) {
    if (info.enum_count > 0) return kAudioUnitParameterUnit_Indexed;
    if (info.curve == PM_PARAM_CURVE_BOOLEAN) return kAudioUnitParameterUnit_Boolean;
    if (info.curve == PM_PARAM_CURVE_DB || strcmp(info.unit, "dB") == 0) return kAudioUnitParameterUnit_Decibels;
    if (strcmp(info.unit, "s") == 0) return kAudioUnitParameterUnit_Seconds;
    if (strcmp(info.unit, "Hz") == 0) return kAudioUnitParameterUnit_Hertz;
    if (strcmp(info.unit, "frames") == 0) return kAudioUnitParameterUnit_SampleFrames;
    if (info.unit[0] != '\0') return kAudioUnitParameterUnit_CustomUnit;
    return kAudioUnitParameterUnit_Generic;
}

NSError* osError(OSStatus status) {
    return [NSError errorWithDomain:NSOSStatusErrorDomain code:status userInfo:nil];
}

}  // namespace

@implementation PianoAudioUnit {
    pm_au::RenderState* _state;
    dispatch_queue_t _queue;  // serial: every non-real-time call into the host
    AUAudioUnitBus* _outputBus;
    AUAudioUnitBusArray* _outputBusArray;
    AUParameterTree* _tree;
    AUAudioUnitPreset* _currentPreset;
    NSArray<AUAudioUnitPreset*>* _factoryPresets;
    std::atomic<bool> _applyingState;  // tree refresh: do not echo values back into the engine
    // _queue only:
    BOOL _resourcesAllocated;
    BOOL _prepareScheduled;
    int32_t _preparedFrames;
}

- (nullable instancetype)initWithComponentDescription:(AudioComponentDescription)componentDescription
                                              options:(AudioComponentInstantiationOptions)options
                                                error:(NSError**)outError {
    self = [super initWithComponentDescription:componentDescription options:options error:outError];
    if (!self) return nil;

    _state = new (std::nothrow) pm_au::RenderState();
    if (_state) _state->host = pm_host_create();
    if (!_state || !_state->host) {
        if (outError) *outError = osError(kAudioUnitErr_FailedInitialization);
        return nil;
    }
    _state->engine = pm_host_engine(_state->host);
    pm_au::classifyParams(*_state);
    pm_host_set_monitor_enabled(_state->host, 0);  // nobody drains the monitor ring in the AU
    _queue = dispatch_queue_create("piano.au.nonrealtime", DISPATCH_QUEUE_SERIAL);
    _applyingState.store(false);

    AVAudioFormat* format = [[AVAudioFormat alloc] initStandardFormatWithSampleRate:48000.0 channels:2];
    _outputBus = [[AUAudioUnitBus alloc] initWithFormat:format error:outError];
    if (!_outputBus) return nil;
    _outputBus.maximumChannelCount = 2;
    _outputBusArray = [[AUAudioUnitBusArray alloc] initWithAudioUnit:self
                                                             busType:AUAudioUnitBusTypeOutput
                                                              busses:@[ _outputBus ]];

    _factoryPresets = @[ [AUAudioUnitPreset new] ];
    _factoryPresets[0].number = 0;
    _factoryPresets[0].name = @"Default";
    _currentPreset = _factoryPresets[0];

    [self buildParameterTree];
    self.maximumFramesToRender = 512;
    return self;
}

- (void)dealloc {
    if (_state) {
        pm_host_destroy(_state->host);
        std::free(_state->scratchLeft);
        std::free(_state->scratchRight);
        delete _state;
    }
}

#pragma mark - Parameters

- (void)buildParameterTree {
    NSMutableArray<AUParameter*>* params = [NSMutableArray array];
    const int32_t count = pm_param_count();
    for (int32_t i = 0; i < count; ++i) {
        pm_param_info_t info;
        if (!pm_param_info(i, &info) || (info.flags & PM_PARAM_FLAG_HIDDEN)) continue;
        AudioUnitParameterOptions flags = kAudioUnitParameterFlag_IsReadable | kAudioUnitParameterFlag_IsWritable;
        if (info.flags & PM_PARAM_FLAG_SMOOTHED) flags |= kAudioUnitParameterFlag_CanRamp;
        if (info.cls == PM_PARAM_CLASS_PREPARE || (info.flags & PM_PARAM_FLAG_NON_REALTIME_SET))
            flags |= kAudioUnitParameterFlag_NonRealTime;
        if (info.curve == PM_PARAM_CURVE_LOG) flags |= kAudioUnitParameterFlag_DisplayLogarithmic;
        const AudioUnitParameterUnit unit = unitFor(info);
        NSMutableArray<NSString*>* valueStrings = nil;
        if (info.enum_count > 0 && info.enum_names) {
            valueStrings = [NSMutableArray arrayWithCapacity:(NSUInteger)info.enum_count];
            for (int32_t k = 0; k < info.enum_count; ++k)
                [valueStrings addObject:@(info.enum_names[k] ? info.enum_names[k] : "")];
        }
        NSString* unitName = unit == kAudioUnitParameterUnit_CustomUnit ? @(info.unit) : nil;
        AUParameter* p = [AUParameterTree createParameterWithIdentifier:@(info.key)
                                                                   name:@(info.name)
                                                                address:(AUParameterAddress)info.id
                                                                    min:(AUValue)info.min
                                                                    max:(AUValue)info.max
                                                                   unit:unit
                                                               unitName:unitName
                                                                  flags:flags
                                                           valueStrings:valueStrings
                                                    dependentParameters:nil];
        [params addObject:p];
    }
    _tree = [AUParameterTree createTreeWithChildren:params];

    __weak PianoAudioUnit* weakSelf = self;
    _tree.implementorValueObserver = ^(AUParameter* param, AUValue value) {
        PianoAudioUnit* s = weakSelf;
        if (!s || s->_applyingState.load(std::memory_order_relaxed)) return;
        const pm_au::ParamKind kind = pm_au::paramKind(*s->_state, (int64_t)param.address);
        if (kind == pm_au::kParamRealtime) {
            pm_engine_set_param(s->_state->engine, (int32_t)param.address, (double)value);
        } else if (kind != pm_au::kParamUnknown) {
            // The observer may run on any host thread; NON_REALTIME_SET and
            // class B parameters go through the serial queue instead.
            [s setNonRealtimeParam:(int32_t)param.address value:(double)value kind:kind];
        }
    };
    _tree.implementorValueProvider = ^AUValue(AUParameter* param) {
        PianoAudioUnit* s = weakSelf;
        double v = 0.0;
        if (s) pm_engine_get_param(s->_state->engine, (int32_t)param.address, &v);
        return (AUValue)v;
    };
}

- (AUParameterTree*)parameterTree {
    return _tree;
}

- (void)setNonRealtimeParam:(int32_t)pid value:(double)value kind:(pm_au::ParamKind)kind {
    __weak PianoAudioUnit* weakSelf = self;
    dispatch_async(_queue, ^{
        PianoAudioUnit* s = weakSelf;
        if (!s) return;
        pm_engine* engine = s->_state->engine;
        double before = 0.0, after = 0.0;
        pm_engine_get_param(engine, pid, &before);
        pm_engine_set_param(engine, pid, value);
        pm_engine_get_param(engine, pid, &after);
        if (kind == pm_au::kParamPrepare && before != after) [s schedulePrepare];
    });
}

// _queue only. A class B change re-prepares the host (which resets all
// sound), so a burst of changes (a knob drag, a host restoring parameters
// one by one) is coalesced into one prepare 100 ms after the first.
- (void)schedulePrepare {
    if (_prepareScheduled) return;
    _prepareScheduled = YES;
    __weak PianoAudioUnit* weakSelf = self;
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 100 * NSEC_PER_MSEC), _queue, ^{
        PianoAudioUnit* s = weakSelf;
        if (!s) return;
        s->_prepareScheduled = NO;
        pm_host* host = s->_state->host;
        if (s->_resourcesAllocated && pm_host_is_prepared(host))
            pm_host_prepare(host, pm_host_sample_rate(host), s->_preparedFrames);
    });
}

// Tells the tree's observers (host UI, automation) about values that changed
// inside the engine, without sending them back to the engine.
- (void)refreshParameterTree {
    [self willChangeValueForKey:@"allParameterValues"];
    _applyingState.store(true, std::memory_order_relaxed);
    for (AUParameter* p in _tree.allParameters) {
        double v = 0.0;
        if (pm_engine_get_param(_state->engine, (int32_t)p.address, &v)) [p setValue:(AUValue)v originator:nil];
    }
    _applyingState.store(false, std::memory_order_relaxed);
    [self didChangeValueForKey:@"allParameterValues"];
}

#pragma mark - State and presets

- (nullable NSData*)presetJSONWithName:(NSString*)name {
    __block NSData* result = nil;
    const char* cname = name.UTF8String;
    dispatch_sync(_queue, ^{
        pm_host* host = self->_state->host;
        const size_t n = pm_host_preset_write(host, cname, PM_PRESET_WRITE_FULL_STATE, nullptr, 0);
        if (n == 0) return;
        NSMutableData* data = [NSMutableData dataWithLength:n + 1];
        const size_t w = pm_host_preset_write(host, cname, PM_PRESET_WRITE_FULL_STATE,
                                              static_cast<char*>(data.mutableBytes), n + 1);
        data.length = w < n ? w : n;
        result = data;
    });
    return result;
}

- (BOOL)applyPresetJSON:(const char*)json length:(size_t)length flags:(int32_t)flags {
    __block int ok = 0;
    dispatch_sync(_queue, ^{
        pm_preset_result_t r;
        // REPREPARE: pm_host_prepare at the current rate and size if the host
        // is prepared; it blocks only this queue, never the audio thread.
        ok = pm_host_preset_read(self->_state->host, json, length, flags | PM_PRESET_READ_REPREPARE, &r);
    });
    if (ok) [self refreshParameterTree];
    return ok != 0;
}

- (NSDictionary<NSString*, id>*)fullState {
    NSMutableDictionary<NSString*, id>* state = [[super fullState] mutableCopy] ?: [NSMutableDictionary dictionary];
    NSData* json = [self presetJSONWithName:_currentPreset.name ?: @""];
    if (json) state[PianoAudioUnitPresetKey] = json;
    return state;
}

- (void)setFullState:(nullable NSDictionary<NSString*, id>*)fullState {
    id json = fullState[PianoAudioUnitPresetKey];
    if (![json isKindOfClass:[NSData class]]) {
        [super setFullState:fullState];
        return;
    }
    // The preset JSON is authoritative; whatever parameter values the base
    // class restores are not applied twice.
    _applyingState.store(true, std::memory_order_relaxed);
    [super setFullState:fullState];
    _applyingState.store(false, std::memory_order_relaxed);
    NSData* data = (NSData*)json;
    [self applyPresetJSON:static_cast<const char*>(data.bytes) length:data.length flags:0];
}

- (NSArray<AUAudioUnitPreset*>*)factoryPresets {
    return _factoryPresets;
}

- (nullable AUAudioUnitPreset*)currentPreset {
    return _currentPreset;
}

- (void)setCurrentPreset:(nullable AUAudioUnitPreset*)preset {
    if (!preset) return;
    if (preset.number == 0) {
        [self applyPresetJSON:kDefaultPresetJSON
                       length:sizeof(kDefaultPresetJSON) - 1
                        flags:PM_PRESET_READ_MISSING_TO_DEFAULT];
        _currentPreset = _factoryPresets[0];
    } else {
        // User presets (negative numbers) are restored by the host through fullState.
        _currentPreset = preset;
    }
}

#pragma mark - Busses and formats

- (AUAudioUnitBusArray*)outputBusses {
    return _outputBusArray;
}

- (NSArray<NSNumber*>*)channelCapabilities {
    return @[ @0, @2 ];  // no input, stereo output
}

- (BOOL)shouldChangeToFormat:(AVAudioFormat*)format forBus:(AUAudioUnitBus*)bus {
    if (format.commonFormat != AVAudioPCMFormatFloat32 || format.isInterleaved || format.channelCount != 2)
        return NO;
    return [super shouldChangeToFormat:format forBus:bus];
}

// MIDI 2.0: note velocities and controllers reach the engine at full
// resolution; hosts that only speak MIDI 1.0 still send AURenderEventMIDI.
- (MIDIProtocolID)audioUnitMIDIProtocol {
    return kMIDIProtocol_2_0;
}

#pragma mark - Render resources

- (BOOL)allocateRenderResourcesAndReturnError:(NSError**)outError {
    if (![super allocateRenderResourcesAndReturnError:outError]) return NO;

    const double rate = _outputBus.format.sampleRate;
    const int32_t maxFrames = (int32_t)self.maximumFramesToRender;
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);

    // The render block is not running between deallocate and allocate.
    std::free(_state->scratchLeft);
    std::free(_state->scratchRight);
    _state->scratchLeft = static_cast<float*>(std::calloc((size_t)maxFrames, sizeof(float)));
    _state->scratchRight = static_cast<float*>(std::calloc((size_t)maxFrames, sizeof(float)));
    _state->maxFrames = maxFrames;
    _state->ticksPerSample = pm_host_ticks_per_sample(rate, timebase.numer, timebase.denom);

    __block int ok = 0;
    dispatch_sync(_queue, ^{
        pm_host_set_timebase(self->_state->host, timebase.numer, timebase.denom);
        ok = pm_host_prepare(self->_state->host, rate, maxFrames);
        self->_resourcesAllocated = ok != 0;
        self->_preparedFrames = maxFrames;
    });
    if (!ok || !_state->scratchLeft || !_state->scratchRight) {
        if (outError) *outError = osError(kAudioUnitErr_FailedInitialization);
        [self deallocateRenderResources];
        return NO;
    }
    return YES;
}

- (void)deallocateRenderResources {
    // The host stays prepared (and keeps its memory) until dealloc.
    dispatch_sync(_queue, ^{
        self->_resourcesAllocated = NO;
    });
    [super deallocateRenderResources];
}

#pragma mark - Render

- (AUInternalRenderBlock)internalRenderBlock {
    // Captures the C++ state only: no Objective-C object is touched, retained
    // or messaged on the audio thread.
    pm_au::RenderState* state = _state;
    return ^AUAudioUnitStatus(AudioUnitRenderActionFlags* actionFlags, const AudioTimeStamp* timestamp,
                              AUAudioFrameCount frameCount, NSInteger outputBusNumber,
                              AudioBufferList* outputData, const AURenderEvent* realtimeEventListHead,
                              AURenderPullInputBlock __unsafe_unretained pullInputBlock) {
        (void)actionFlags;
        (void)pullInputBlock;
        if (outputBusNumber != 0 || !outputData) return noErr;
        if ((int64_t)frameCount > (int64_t)state->maxFrames) return kAudioUnitErr_TooManyFramesToProcess;
        const int32_t frames = (int32_t)frameCount;

        // mach_absolute_time is the clock pm_host was given; it is a
        // real-time-safe register read where the host gives no host time.
        const uint64_t hostTime =
            (timestamp->mFlags & kAudioTimeStampHostTimeValid) ? timestamp->mHostTime : mach_absolute_time();
        const bool sampleTimeValid = (timestamp->mFlags & kAudioTimeStampSampleTimeValid) != 0;
        const double bufferSampleTime = timestamp->mSampleTime;

        for (const AURenderEvent* e = realtimeEventListHead; e != nullptr; e = e->head.next) {
            const int32_t frame =
                sampleTimeValid ? pm_au::frameOffset(e->head.eventSampleTime, bufferSampleTime, frames) : 0;
            const uint64_t t = pm_au::hostTimeAtFrame(hostTime, frame, state->ticksPerSample);
            switch (e->head.eventType) {
                case AURenderEventParameter:
                case AURenderEventParameterRamp:
                    pm_au::parameterEvent(*state, (int64_t)e->parameter.parameterAddress, e->parameter.value);
                    break;
                case AURenderEventMIDI:
                    pm_au::midiBytes(*state, e->MIDI.data, (int32_t)e->MIDI.length, t, e->MIDI.cable);
                    break;
                case AURenderEventMIDIEventList: {
                    const MIDIEventList* list = &e->MIDIEventsList.eventList;
                    const MIDIEventPacket* packet = &list->packet[0];
                    for (UInt32 i = 0; i < list->numPackets; ++i) {
                        pm_au::midiUmp(*state, packet->words, (int32_t)packet->wordCount, t,
                                       e->MIDIEventsList.cable);
                        packet = MIDIEventPacketNext(packet);
                    }
                    break;
                }
                default:  // SysEx and future event types are ignored
                    break;
            }
        }

        const UInt32 nb = outputData->mNumberBuffers;
        if (nb == 0) return noErr;
        AudioBuffer* buffers = outputData->mBuffers;
        // A null mData asks the AU to provide the memory (it then stays valid
        // until the next render call).
        if (!buffers[0].mData) buffers[0].mData = state->scratchLeft;
        if (nb > 1 && !buffers[1].mData) buffers[1].mData = state->scratchRight;
        for (UInt32 b = 0; b < nb && b < 2; ++b) buffers[b].mDataByteSize = frameCount * (UInt32)sizeof(float);
        pm_au::render(*state, static_cast<float*>(buffers[0].mData),
                      nb > 1 ? static_cast<float*>(buffers[1].mData) : nullptr, frames, hostTime);
        return noErr;
    };
}

@end
