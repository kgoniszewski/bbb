#pragma once

#include <JuceHeader.h>

class HighGainAmpAudioProcessor : public juce::AudioProcessor
{
public:
    HighGainAmpAudioProcessor();
    ~HighGainAmpAudioProcessor() override;

    // AudioProcessor overrides
    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

   #if ! JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported(const BusesLayout& layouts) const override;
   #endif

    void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return JucePlugin_Name; }

    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram(int) override {}
    const juce::String getProgramName(int) override { return {}; }
    void changeProgramName(int, const juce::String&) override {}

    void getStateInformation(juce::MemoryBlock& destData) override;
    void setStateInformation(const void* data, int sizeInBytes) override;

    // Parameter access
    juce::AudioProcessorValueTreeState& getAPVTS() { return apvts; }

    // IR loading from UI
    void loadImpulseResponse(const juce::File& file);
    juce::String getLoadedIRName() const { return loadedIRFile.existsAsFile() ? loadedIRFile.getFileName() : juce::String("No IR"); }

    // Parameter IDs
    static constexpr const char* PARAM_GAIN = "gain_dB";
    static constexpr const char* PARAM_BASS = "bass_dB";
    static constexpr const char* PARAM_MID = "mid_dB";
    static constexpr const char* PARAM_TREBLE = "treble_dB";
    static constexpr const char* PARAM_PRESENCE = "presence_dB";
    static constexpr const char* PARAM_VOLUME = "volume_dB";

private:
    // Parameter layout
    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

    void updateAllDSP();
    void updateToneStack(double osSampleRate);

    // State
    juce::AudioProcessorValueTreeState apvts;

    // DSP
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampling; // 4x
    juce::dsp::WaveShaper<float> waveshaper;

    juce::dsp::IIR::Filter<float> lowShelfFilter;    // Bass
    juce::dsp::IIR::Filter<float> midPeakFilter;     // Mid
    juce::dsp::IIR::Filter<float> highShelfFilter;   // Treble
    juce::dsp::IIR::Filter<float> presenceShelfFilter; // Presence (higher freq)

    juce::dsp::Convolution cabConvolution;

    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> preGainLinear;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> postGainLinear;

    juce::File loadedIRFile;
    bool irLoaded { false };

    double currentSampleRate { 44100.0 };
    int currentBlockSize { 512 };

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(HighGainAmpAudioProcessor)
};