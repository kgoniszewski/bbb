#include "PluginProcessor.h"
#include "PluginEditor.h"

using juce::AudioParameterFloat;
using juce::NormalisableRange;

namespace {
    inline float decibelsToLinearSmoothed(juce::SmoothedValue<float>& target, float dB)
    {
        target.setTargetValue(juce::Decibels::decibelsToGain(dB));
        return target.getNextValue();
    }

    inline float decibelsToLinear(float dB)
    {
        return juce::Decibels::decibelsToGain(dB);
    }
}

HighGainAmpAudioProcessor::HighGainAmpAudioProcessor()
    : AudioProcessor (BusesProperties()
                        .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                        .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts(*this, nullptr, "PARAMS", createParameterLayout())
{
    preGainLinear.reset(44100.0, 0.02);
    postGainLinear.reset(44100.0, 0.02);

    // Oversampling: 4x, IIR halfband
    oversampling = std::make_unique<juce::dsp::Oversampling<float>>(2 /* numChannels */, 2 /* numStages (2 -> 4x) */, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, true);

    // High-gain shaper: use tanh with strong drive
    waveshaper.functionToUse = [] (float x) {
        const float drive = 3.5f; // base shape; actual drive controlled by pre-gain
        return std::tanh(drive * x);
    };
}

HighGainAmpAudioProcessor::~HighGainAmpAudioProcessor() = default;

void HighGainAmpAudioProcessor::prepareToPlay(double sampleRate, int samplesPerBlock)
{
    currentSampleRate = sampleRate;
    currentBlockSize  = samplesPerBlock;

    preGainLinear.reset(sampleRate, 0.02);
    postGainLinear.reset(sampleRate, 0.02);

    juce::dsp::ProcessSpec spec;
    spec.sampleRate = sampleRate;
    spec.maximumBlockSize = static_cast<juce::uint32>(samplesPerBlock);
    spec.numChannels = static_cast<juce::uint32>(getTotalNumOutputChannels());

    cabConvolution.reset();
    cabConvolution.prepare(spec);

    oversampling->reset();
    oversampling->initProcessing(static_cast<size_t>(samplesPerBlock));

    // Prepare filters at OS rate
    juce::dsp::ProcessSpec osSpec = spec;
    osSpec.sampleRate = sampleRate * oversampling->getOversamplingFactor();

    lowShelfFilter.prepare(osSpec);
    midPeakFilter.prepare(osSpec);
    highShelfFilter.prepare(osSpec);
    presenceShelfFilter.prepare(osSpec);

    lowShelfFilter.reset();
    midPeakFilter.reset();
    highShelfFilter.reset();
    presenceShelfFilter.reset();

    updateAllDSP();
}

void HighGainAmpAudioProcessor::releaseResources() {}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new HighGainAmpAudioProcessor();
}

#if ! JucePlugin_PreferredChannelConfigurations
bool HighGainAmpAudioProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const
{
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    if (layouts.getMainInputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    return true;
}
#endif

void HighGainAmpAudioProcessor::updateToneStack(double osSampleRate)
{
    auto& params = getAPVTS();

    const auto bass = params.getRawParameterValue(PARAM_BASS)->load();
    const auto mid = params.getRawParameterValue(PARAM_MID)->load();
    const auto treble = params.getRawParameterValue(PARAM_TREBLE)->load();
    const auto presence = params.getRawParameterValue(PARAM_PRESENCE)->load();

    // Tone stack design: shelves + mid peak
    const double bassFreq = 100.0;
    const double midFreq = 700.0;
    const double trebleFreq = 3500.0;
    const double presenceFreq = 4500.0;

    auto lowCoeffs = juce::dsp::IIR::Coefficients<float>::makeLowShelf(osSampleRate, bassFreq, 0.707f, decibelsToLinear(bass));
    auto midCoeffs = juce::dsp::IIR::Coefficients<float>::makePeakFilter(osSampleRate, midFreq, 0.8f, decibelsToLinear(mid));
    auto highCoeffs = juce::dsp::IIR::Coefficients<float>::makeHighShelf(osSampleRate, trebleFreq, 0.707f, decibelsToLinear(treble));
    auto presenceCoeffs = juce::dsp::IIR::Coefficients<float>::makeHighShelf(osSampleRate, presenceFreq, 0.6f, decibelsToLinear(presence));

    lowShelfFilter.coefficients = lowCoeffs;
    midPeakFilter.coefficients = midCoeffs;
    highShelfFilter.coefficients = highCoeffs;
    presenceShelfFilter.coefficients = presenceCoeffs;
}

void HighGainAmpAudioProcessor::updateAllDSP()
{
    const auto gainDB = apvts.getRawParameterValue(PARAM_GAIN)->load();
    const auto volDB  = apvts.getRawParameterValue(PARAM_VOLUME)->load();

    preGainLinear.setTargetValue(juce::Decibels::decibelsToGain(gainDB));
    postGainLinear.setTargetValue(juce::Decibels::decibelsToGain(volDB));

    updateToneStack(currentSampleRate * oversampling->getOversamplingFactor());
}

void HighGainAmpAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;

    const int numInputChannels  = getTotalNumInputChannels();
    const int numOutputChannels = getTotalNumOutputChannels();

    for (int ch = numInputChannels; ch < numOutputChannels; ++ch)
        buffer.clear(ch, 0, buffer.getNumSamples());

    auto audioBlock = juce::dsp::AudioBlock<float>(buffer);

    // Up-sample
    auto oversampledBlock = oversampling->processSamplesUp(audioBlock);

    // Apply pre-gain, waveshaper, tone stack, presence at OS rate
    auto contextOS = juce::dsp::ProcessContextReplacing<float>(oversampledBlock);

    // Pre-gain per-sample smoothed
    for (size_t ch = 0; ch < oversampledBlock.getNumChannels(); ++ch)
    {
        auto* data = oversampledBlock.getChannelPointer(ch);
        for (size_t n = 0; n < oversampledBlock.getNumSamples(); ++n)
            data[n] *= preGainLinear.getNextValue();
    }

    waveshaper.process(contextOS);

    lowShelfFilter.process(contextOS);
    midPeakFilter.process(contextOS);
    highShelfFilter.process(contextOS);
    presenceShelfFilter.process(contextOS);

    // Down-sample back to project rate
    oversampling->processSamplesDown(audioBlock);

    // Convolution IR (cab) at project rate
    auto block = juce::dsp::AudioBlock<float>(buffer);
    juce::dsp::ProcessContextReplacing<float> context(block);
    cabConvolution.process(context);

    // Post volume
    const float post = postGainLinear.getNextValue();
    for (int ch = 0; ch < buffer.getNumChannels(); ++ch)
        buffer.applyGain(ch, 0, buffer.getNumSamples(), post);
}

juce::AudioProcessorEditor* HighGainAmpAudioProcessor::createEditor()
{
    return new HighGainAmpAudioProcessorEditor(*this);
}

void HighGainAmpAudioProcessor::getStateInformation(juce::MemoryBlock& destData)
{
    auto state = apvts.copyState();

    if (loadedIRFile.existsAsFile())
        state.setProperty("irPath", loadedIRFile.getFullPathName(), nullptr);

    juce::MemoryOutputStream mos(destData, false);
    state.writeToStream(mos);
}

void HighGainAmpAudioProcessor::setStateInformation(const void* data, int sizeInBytes)
{
    juce::ValueTree state = juce::ValueTree::readFromData(data, static_cast<size_t>(sizeInBytes));
    if (state.isValid())
    {
        apvts.replaceState(state);

        auto irPath = state.getProperty("irPath").toString();
        if (irPath.isNotEmpty())
        {
            juce::File f(irPath);
            if (f.existsAsFile())
                loadImpulseResponse(f);
        }
    }
    updateAllDSP();
}

juce::AudioProcessorValueTreeState::ParameterLayout HighGainAmpAudioProcessor::createParameterLayout()
{
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> params;

    auto dBRangeWide = NormalisableRange<float>(-12.0f, 12.0f, 0.01f);
    auto dBRangeGain = NormalisableRange<float>(0.0f, 30.0f, 0.01f);
    auto dBRangeVol  = NormalisableRange<float>(-60.0f, 12.0f, 0.01f);

    params.push_back(std::make_unique<AudioParameterFloat>(PARAM_GAIN,     "Gain",     dBRangeGain, 18.0f));
    params.push_back(std::make_unique<AudioParameterFloat>(PARAM_BASS,     "Bass",     dBRangeWide, 0.0f));
    params.push_back(std::make_unique<AudioParameterFloat>(PARAM_MID,      "Mid",      dBRangeWide, 0.0f));
    params.push_back(std::make_unique<AudioParameterFloat>(PARAM_TREBLE,   "Treble",   dBRangeWide, 0.0f));
    params.push_back(std::make_unique<AudioParameterFloat>(PARAM_PRESENCE, "Presence", dBRangeWide, 0.0f));
    params.push_back(std::make_unique<AudioParameterFloat>(PARAM_VOLUME,   "Volume",   dBRangeVol, -6.0f));

    return { params.begin(), params.end() };
}

void HighGainAmpAudioProcessor::loadImpulseResponse(const juce::File& file)
{
    if (! file.existsAsFile())
        return;

    cabConvolution.reset();
    cabConvolution.loadImpulseResponse(file,
        juce::dsp::Convolution::Stereo::yes,
        juce::dsp::Convolution::Trim::no,
        0,
        juce::dsp::Convolution::Normalise::yes);

    loadedIRFile = file;
    irLoaded = true;
}