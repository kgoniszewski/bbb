#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

class HighGainAmpAudioProcessorEditor : public juce::AudioProcessorEditor
{
public:
    explicit HighGainAmpAudioProcessorEditor(HighGainAmpAudioProcessor&);
    ~HighGainAmpAudioProcessorEditor() override;

    void paint(juce::Graphics&) override;
    void resized() override;

private:
    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;

    void initKnob(juce::Slider& slider, const juce::String& name);

    HighGainAmpAudioProcessor& processor;

    juce::Slider gainSlider, bassSlider, midSlider, trebleSlider, presenceSlider, volumeSlider;

    std::unique_ptr<SliderAttachment> gainAttachment, bassAttachment, midAttachment, trebleAttachment, presenceAttachment, volumeAttachment;

    juce::Label titleLabel;
    juce::TextButton loadIRButton { "Load IR" };
    juce::Label irNameLabel;
    std::unique_ptr<juce::FileChooser> fileChooser;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(HighGainAmpAudioProcessorEditor)
};