// AUv3 music device wrapping a pm_host (plan §5.1, §5.2, §5.3). Shared by the
// app extension and, later, the app (in-process hosting).
#import <AudioToolbox/AudioToolbox.h>
#import <AVFAudio/AVFAudio.h>

NS_ASSUME_NONNULL_BEGIN

// Component codes; must match AudioComponents in the extension's Info.plist
// (apps/PianoApp/project.yml).
#define PIANO_AU_TYPE 'aumu'
#define PIANO_AU_SUBTYPE 'Pmd1'
#define PIANO_AU_MANUFACTURER 'Pmdl'

// Key of the preset JSON (NSData, UTF-8) inside fullState.
extern NSString* const PianoAudioUnitPresetKey;

@interface PianoAudioUnit : AUAudioUnit
@end

NS_ASSUME_NONNULL_END
