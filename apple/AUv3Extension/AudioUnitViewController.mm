#import "AudioUnitViewController.h"

#import "PianoAudioUnit.h"

#if TARGET_OS_IPHONE
#import <UIKit/UIKit.h>
#else
#import <AppKit/AppKit.h>
#endif

@implementation AudioUnitViewController {
    AUAudioUnit* _audioUnit;
}

- (nullable AUAudioUnit*)createAudioUnitWithComponentDescription:(AudioComponentDescription)desc
                                                           error:(NSError**)error {
    _audioUnit = [[PianoAudioUnit alloc] initWithComponentDescription:desc error:error];
    return _audioUnit;
}

#if TARGET_OS_IPHONE
- (void)viewDidLoad {
    [super viewDidLoad];
    self.view.backgroundColor = UIColor.systemBackgroundColor;
    UILabel* label = [[UILabel alloc] init];
    label.text = @"Piano Model: Concert Grand";
    label.textColor = UIColor.labelColor;
    label.translatesAutoresizingMaskIntoConstraints = NO;
    [self.view addSubview:label];
    [NSLayoutConstraint activateConstraints:@[
        [label.centerXAnchor constraintEqualToAnchor:self.view.centerXAnchor],
        [label.centerYAnchor constraintEqualToAnchor:self.view.centerYAnchor],
    ]];
    self.preferredContentSize = CGSizeMake(400, 120);
}
#else
- (void)loadView {
    // No nib: an AUViewController on macOS must create its own view.
    NSView* view = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 400, 120)];
    NSTextField* label = [NSTextField labelWithString:@"Piano Model: Concert Grand"];
    label.translatesAutoresizingMaskIntoConstraints = NO;
    [view addSubview:label];
    [NSLayoutConstraint activateConstraints:@[
        [label.centerXAnchor constraintEqualToAnchor:view.centerXAnchor],
        [label.centerYAnchor constraintEqualToAnchor:view.centerYAnchor],
    ]];
    self.view = view;
    self.preferredContentSize = NSMakeSize(400, 120);
}
#endif

@end
