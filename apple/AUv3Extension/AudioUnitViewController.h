// Principal class of the AUv3 app extension (NSExtensionPrincipalClass):
// creates PianoAudioUnit for the host and shows a minimal view; hosts offer
// their generic parameter UI on top of it.
#import <CoreAudioKit/CoreAudioKit.h>

NS_ASSUME_NONNULL_BEGIN

@interface AudioUnitViewController : AUViewController <AUAudioUnitFactory>
@end

NS_ASSUME_NONNULL_END
