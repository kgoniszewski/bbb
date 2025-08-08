# HighGainAmp (JUCE)

High-gain guitar amp plugin with impulse-response (IR) cabinet loader.

- Formats: AU, VST3
- Controls: Gain, Bass, Mid, Treble, Presence, Volume
- DSP: 4x oversampled waveshaper preamp, simple tone stack, optional IR convolution

## Build (macOS)

Requirements:
- Xcode (or command line tools)
- CMake >= 3.22

This project fetches JUCE automatically via CMake's FetchContent.

```bash
# From project root
cmake -B build -S . -G Xcode -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"
cmake --build build --config Release
```

Produced binaries will be located under `build/Release/` (Xcode generator) or under the default CMake build dir for other generators.

To build with Ninja:

```bash
cmake -B build -S . -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

## Using the Plugin

- Load the plugin in your DAW.
- Click "Load IR" and choose a cabinet IR (WAV/AIFF). The IR path is stored in the plugin state.
- Adjust the six knobs. "Gain" sets preamp drive; "Volume" sets output level; the rest are tone controls.

## Notes

- The oversampling is applied around the non-linear stage and tone stack, then downsampled before convolution for efficiency.
- The IR loader uses JUCE's `dsp::Convolution` with normalisation.

## License

This example uses JUCE under its license. See JUCE's license terms.