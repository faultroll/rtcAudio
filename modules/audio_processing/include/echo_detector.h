
#ifndef MODULES_AUDIO_PROCESSING_INCLUDE_ECHO_DETECTOR_H_
#define MODULES_AUDIO_PROCESSING_INCLUDE_ECHO_DETECTOR_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "rtc_base/view.h"
#include "rtc_base/ref_count.h"

namespace webrtc {

class AudioBuffer;

// Interface for an echo detector submodule.
class EchoDetector : public rtc::RefCountInterface {
 public:
  // (Re-)Initializes the submodule.
  virtual void Initialize(int capture_sample_rate_hz,
                          int num_capture_channels,
                          int render_sample_rate_hz,
                          int num_render_channels) = 0;

  // Analysis (not changing) of the render signal.
  virtual void AnalyzeRenderAudio(RTC_VIEW(const float) render_audio) = 0;

  // Analysis (not changing) of the capture signal.
  virtual void AnalyzeCaptureAudio(
      RTC_VIEW(const float) capture_audio) = 0;

  // Pack an AudioBuffer into a vector<float>.
  static void PackRenderAudioBuffer(AudioBuffer* audio,
                                    std::vector<float>* packed_buffer);

  struct Metrics {
    double echo_likelihood;
    double echo_likelihood_recent_max;
  };

  // Collect current metrics from the echo detector.
  virtual Metrics GetMetrics() const = 0;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_INCLUDE_ECHO_DETECTOR_H_
