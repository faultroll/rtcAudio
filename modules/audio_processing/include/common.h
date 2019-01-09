
#ifndef MODULES_AUDIO_PROCESSING_COMMON_H_
#define MODULES_AUDIO_PROCESSING_COMMON_H_

// #include <vector>
// #include <memory> // std::unique_ptr

#include "rtc_base/arraysize.h"

namespace webrtc {
namespace AudioProcessing {

  enum Error {
    // Fatal errors.
    kNoError = 0,
    kUnspecifiedError = -1,
    kCreationFailedError = -2,
    kUnsupportedComponentError = -3,
    kUnsupportedFunctionError = -4,
    kNullPointerError = -5,
    kBadParameterError = -6,
    kBadSampleRateError = -7,
    kBadDataLengthError = -8,
    kBadNumberChannelsError = -9,
    kFileError = -10,
    kStreamParameterNotSetError = -11,
    kNotEnabledError = -12,

    // Warnings are non-fatal.
    // This results when a set_stream_ parameter is out of range. Processing
    // will continue, but the parameter may have been truncated.
    kBadStreamParameterWarning = -13
  };

  enum NativeRate {
    kSampleRate8kHz = 8000,
    kSampleRate16kHz = 16000,
    kSampleRate32kHz = 32000,
    kSampleRate48kHz = 48000
  };

  // TODO(kwiberg): We currently need to support a compiler (Visual C++) that
  // complains if we don't explicitly state the size of the array here. Remove
  // the size when that's no longer the case.
  static constexpr int kNativeSampleRatesHz[4] = {
      kSampleRate8kHz, kSampleRate16kHz, kSampleRate32kHz, kSampleRate48kHz};
  static constexpr size_t kNumNativeSampleRates =
      arraysize(kNativeSampleRatesHz);
  static constexpr int kMaxNativeSampleRateHz =
      kNativeSampleRatesHz[kNumNativeSampleRates - 1];

  static const int kChunkSizeMs = 10;

}  // namespace AudioProcessing


// Module interface in audio processing 
// APM operates on two audio streams on a frame-by-frame basis. Frames of the 
// primary stream, on which all processing is applied, are passed to 
// |ProcessStream()|. Frames of the reverse direction stream are passed to 
// |ProcessReverseStream()|. 
// On the client-side, this will typically be the 
// near-end (capture) and far-end (render) streams, respectively.
// On the server-side, the reverse stream will normally 
// not be used, with processing occurring on each incoming stream.
// class AudioFrame;
class AudioBuffer;
// struct Metrics;
class ApmCaptureModule {
 public:
  // returns AudioProcessing::Error
  // Analysis (not changing) of the capture signal.
  virtual int AnalyzeCaptureAudio(AudioBuffer* audio_buffer) = 0;
  // Processes the capture signal.
  virtual int ProcessCaptureAudio(AudioBuffer* audio_buffer) = 0;
  /* virtual int ProcessCaptureAudio(AudioFrame* audio_frame) {
    if (!audio_buffer_.get()) {
      audio_buffer_.reset(
        new AudioBuffer(audio_frame->sample_rate_hz(), audio_frame->num_channels(),
                        audio_frame->sample_rate_hz(), audio_frame->num_channels(),
                        audio_frame->sample_rate_hz(), audio_frame->num_channels()));
    }

    audio_buffer_->DeinterleaveFrom(audio_frame);
    audio_buffer_->SplitIntoFrequencyBands();
    const int error = ProcessCaptureAudio(audio_buffer_.get());
    audio_buffer_->MergeFrequencyBands();
    audio_buffer_->InterleaveTo(audio_frame, false);

    return error;
  } */

  // Collect current metrics from the module.
  // virtual int GetMetrics(Metrics* metrics) const = 0;

 /* private:
  std::unique_ptr<AudioBuffer> audio_buffer_; */
};
class ApmRenderModule {
 public:
  // see |ApmCaptureModule|
  // Analysis (not changing) of the render signal.
  virtual int AnalyzeRenderAudio(AudioBuffer* audio_buffer) = 0;
  // Processes the render signal.
  virtual int ProcessRenderAudio(AudioBuffer* audio_buffer) = 0;
  // Pack an AudioBuffer into a vector<float>.
  // Cannot be declared both virtual and static
  // virtual static void PackRenderAudioBuffer(AudioBuffer* audio_buffer,
  //                                           std::vector<int16_t>* packed_buffer) = 0;
  // virtual int ProcessRenderAudio(RTC_VIEW(const int16_t) packed_buffer) = 0;
  // virtual static void PackRenderAudioBuffer(AudioBuffer* audio_buffer,
  //                                           std::vector<float>* packed_buffer) = 0;
  // virtual int ProcessRenderAudio(RTC_VIEW(const float) packed_buffer) = 0;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_COMMON_H_
