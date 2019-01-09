
#ifndef MODULES_AUDIO_PROCESSING_INCLUDE_NOISE_SUPPRESSION_H_
#define MODULES_AUDIO_PROCESSING_INCLUDE_NOISE_SUPPRESSION_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include <vector>

#include "modules/audio_processing/include/common.h"

namespace webrtc {

// The noise suppression (NS) component attempts to remove noise while
// retaining speech. Recommended to be enabled on the client-side.
//
// Recommended to be enabled on the client-side.
class NoiseSuppression : public ApmCaptureModule {
 public:
  virtual int Enable(bool enable) = 0;
  virtual bool is_enabled() const = 0;

  // Determines the aggressiveness of the suppression. Increasing the level
  // will reduce the noise level at the expense of a higher speech distortion.
  enum Level { kLow, kModerate, kHigh, kVeryHigh };

  virtual int set_level(Level level) = 0;
  virtual Level level() const = 0;

  // Returns the internally computed prior speech probability of current frame
  // averaged over output channels. This is not supported in fixed point, for
  // which |kUnsupportedFunctionError| is returned.
  virtual float speech_probability() const = 0;

  // Returns the noise estimate per frequency bin averaged over all channels.
  virtual std::vector<float> NoiseEstimate() = 0;

  // ApmSubmodule
  virtual void Initialize(size_t num_channels, int sample_rate_hz) = 0;

 // protected:
  virtual ~NoiseSuppression() {}
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_INCLUDE_NOISE_SUPPRESSION_H_
