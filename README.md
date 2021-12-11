
## 

*NOT* test WEBRTC_WIN version

- based on https://github.com/open-webrtc-toolkit/owt-deps-webrtc/tree/70-sdk , https://github.com/open-webrtc-toolkit/owt-deps-webrtc/tree/59-server and https://gitlab.freedesktop.org/pulseaudio/webrtc-audio-processing/-/tree/v1.0
- add post-processing part like mixer/(part of)neteq/...
- try to make it more c like for embed use (eg. pure-c version)
    1. checks (in rtc_base, c part)
    2. spl
    3. vad-common/vad-isac/agc-legacy/ns-core/aec(m/core, delay_estimter)
    4. (cxx) rnn-vad/agc2/ns2/aec3 (and rtc_base, cxx part)
    5. (cxx) apm, mixer, neteq, ...


## 

- apm
    ``` cxx
    // APM operates on two audio streams on a frame-by-frame basis. Frames of the 
    // primary stream, on which all processing is applied, are passed to 
    // |ProcessStream()|. Frames of the reverse direction stream are passed to 
    // |ProcessReverseStream()|. 
    // On the client-side, this will typically be the 
    // near-end (capture) and far-end (render) streams, respectively.
    // On the server-side, the reverse stream will normally 
    // not be used, with processing occurring on each incoming stream.
    ```
    - render
        uses 
        ``` cxx
        std::unique_ptr<SwapQueue<std::vector<float>, RenderQueueItemVerifier<float>>>
        ```
        PackRenderAudioBuffer --> Insert --> Remove --> ProcessRenderAudio
    - capture
        uses
        ``` cxx
        AudioBuffer
        ```
        AnalyzeCaptureAudio --> ProcessCaptureAudio
    render before capture, EmptyQueuedRenderAudio(it calls ProcessRenderAudio) will be called
- agc
    - legacy/WebRtcAgc_xxx(gain applier) --> GainControl(gain applier)
    - LoudnessHistogram --> Agc(level estimator) --> MonoAgc(level estimator) --> AgcManagerDirect(GainControl::set_compression_gain_db)
    Combine these two (old version like m70 agc_manager set gain_control in it)
    ``` cxx
    if (submodules_.agc_manager) {
      submodules_.agc_manager->Process(capture_buffer);
  
      rtc::Optional<int> new_digital_gain =
          submodules_.agc_manager->GetDigitalComressionGain();
      if (new_digital_gain && submodules_.gain_control) {
        submodules_.gain_control->set_compression_gain_db(*new_digital_gain);
      }
    }

    if (submodules_.gain_control) {
      // TODO(peah): Add reporting from AEC3 whether there is echo.
      RETURN_ON_ERR(submodules_.gain_control->ProcessCaptureAudio(
          capture_buffer, /*stream_has_echo*/ false));
    }
    ```
    gain_control_for_experimental_agc using VolumeCallbacks in agc_manager_direct which is deprecated (use set_stream_analog_level instead)
