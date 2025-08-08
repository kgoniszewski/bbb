#include "PluginEditor.h"
#include "PluginProcessor.h"

HighGainAmpAudioProcessorEditor::HighGainAmpAudioProcessorEditor(HighGainAmpAudioProcessor& p)
    : AudioProcessorEditor(&p), processor(p)
{
    setResizable(false, false);
    setSize(640, 380);

    titleLabel.setText("HighGainAmp", juce::dontSendNotification);
    titleLabel.setFont(juce::Font(20.0f, juce::Font::bold));
    titleLabel.setJustificationType(juce::Justification::centred);
    addAndMakeVisible(titleLabel);

    initKnob(gainSlider,     "Gain");
    initKnob(bassSlider,     "Bass");
    initKnob(midSlider,      "Mid");
    initKnob(trebleSlider,   "Treble");
    initKnob(presenceSlider, "Presence");
    initKnob(volumeSlider,   "Volume");

    gainAttachment     = std::make_unique<SliderAttachment>(processor.getAPVTS(), HighGainAmpAudioProcessor::PARAM_GAIN,     gainSlider);
    bassAttachment     = std::make_unique<SliderAttachment>(processor.getAPVTS(), HighGainAmpAudioProcessor::PARAM_BASS,     bassSlider);
    midAttachment      = std::make_unique<SliderAttachment>(processor.getAPVTS(), HighGainAmpAudioProcessor::PARAM_MID,      midSlider);
    trebleAttachment   = std::make_unique<SliderAttachment>(processor.getAPVTS(), HighGainAmpAudioProcessor::PARAM_TREBLE,   trebleSlider);
    presenceAttachment = std::make_unique<SliderAttachment>(processor.getAPVTS(), HighGainAmpAudioProcessor::PARAM_PRESENCE, presenceSlider);
    volumeAttachment   = std::make_unique<SliderAttachment>(processor.getAPVTS(), HighGainAmpAudioProcessor::PARAM_VOLUME,   volumeSlider);

    addAndMakeVisible(loadIRButton);
    loadIRButton.onClick = [this]
    {
        fileChooser = std::make_unique<juce::FileChooser>("Select Impulse Response (wav/aif)", juce::File(), "*.wav;*.aif;*.aiff");
        auto flags = juce::FileBrowserComponent::openMode | juce::FileBrowserComponent::canSelectFiles;
        fileChooser->launchAsync(flags, [this](const juce::FileChooser& fc)
        {
            auto file = fc.getResult();
            if (file.existsAsFile())
            {
                processor.loadImpulseResponse(file);
                irNameLabel.setText(processor.getLoadedIRName(), juce::dontSendNotification);
            }
            fileChooser.reset();
        });
    };

    irNameLabel.setText(processor.getLoadedIRName(), juce::dontSendNotification);
    irNameLabel.setJustificationType(juce::Justification::centred);
    addAndMakeVisible(irNameLabel);
}

HighGainAmpAudioProcessorEditor::~HighGainAmpAudioProcessorEditor() = default;

void HighGainAmpAudioProcessorEditor::initKnob(juce::Slider& s, const juce::String& name)
{
    s.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
    s.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 18);
    s.setColour(juce::Slider::thumbColourId, juce::Colours::orange);
    s.setName(name);
    addAndMakeVisible(s);
}

void HighGainAmpAudioProcessorEditor::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colours::black);
    g.setColour(juce::Colours::white);
}

void HighGainAmpAudioProcessorEditor::resized()
{
    auto area = getLocalBounds().reduced(12);

    auto header = area.removeFromTop(30);
    titleLabel.setBounds(header);

    auto irArea = area.removeFromTop(40);
    loadIRButton.setBounds(irArea.removeFromLeft(120));
    irArea.removeFromLeft(12);
    irNameLabel.setBounds(irArea);

    auto knobRow1 = area.removeFromTop(150);
    auto knobRow2 = area.removeFromTop(150);

    auto placeKnob = [] (juce::Rectangle<int> row, int index, int total) {
        int w = row.getWidth() / total;
        return row.removeFromLeft(w).reduced(8);
    };

    gainSlider.setBounds(placeKnob(knobRow1, 0, 3));
    bassSlider.setBounds(placeKnob(knobRow1, 1, 3));
    midSlider.setBounds(placeKnob(knobRow1, 2, 3));

    trebleSlider.setBounds(placeKnob(knobRow2, 0, 3));
    presenceSlider.setBounds(placeKnob(knobRow2, 1, 3));
    volumeSlider.setBounds(placeKnob(knobRow2, 2, 3));
}