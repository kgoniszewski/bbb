cmake_minimum_required(VERSION 3.22)

project(HighGainAmp VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Fetch JUCE from GitHub (you can also point JUCE_DIR to a local checkout)
FetchContent_Declare(
  juce
  GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
  GIT_TAG 8.0.3
)
FetchContent_MakeAvailable(juce)

# Plugin target
juce_add_plugin(HighGainAmp
    COMPANY_NAME "YourCompany"
    BUNDLE_ID com.yourcompany.HighGainAmp
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD TRUE
    PLUGIN_MANUFACTURER_CODE Your
    PLUGIN_CODE HGam
    FORMATS AU VST3
    PRODUCT_NAME "HighGainAmp"
)

# Sources
target_sources(HighGainAmp PRIVATE
    src/PluginProcessor.cpp
    src/PluginEditor.cpp
    src/PluginProcessor.h
    src/PluginEditor.h
)

target_include_directories(HighGainAmp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# JUCE modules
target_link_libraries(HighGainAmp PRIVATE
    juce::juce_audio_utils
    juce::juce_audio_basics
    juce::juce_audio_devices
    juce::juce_audio_formats
    juce::juce_audio_processors
    juce::juce_core
    juce::juce_dsp
    juce::juce_gui_basics
    juce::juce_gui_extra
)

# Disable web components to slim build
target_compile_definitions(HighGainAmp PRIVATE
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
)

# Enable warnings
if (MSVC)
  target_compile_options(HighGainAmp PRIVATE /W4)
else()
  target_compile_options(HighGainAmp PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Create an AppConfig and BinaryData header as needed
juce_generate_juce_header(HighGainAmp)

# macOS specific settings
if(APPLE)
    set_target_properties(HighGainAmp PROPERTIES
        XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "-"
        XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED "NO"
        XCODE_ATTRIBUTE_DEVELOPMENT_TEAM ""
    )
endif()