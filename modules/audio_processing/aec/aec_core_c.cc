
#include "rtc_base/system/arch.h"
#if !defined(WEBRTC_HAS_SSE2) && !defined(MIPS_FPU_LE) && !defined(WEBRTC_HAS_NEON)

/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

/*
 * The core AEC algorithm, assembly optimization version of speed-critical functions.
 *
 * From aec_core.cc
 */
#include <math.h>
#include <string.h>  // memset

extern "C" {
#include "common_audio/signal_processing/include/signal_processing_library.h"
}
#include "modules/audio_processing/aec/aec_common.h"
#include "modules/audio_processing/aec/aec_core_optimized_methods.h"
#include "common_audio/third_party/ooura/fft_size_128/ooura_fft.h"

// namespace webrtc {

// Matlab code to produce table:
// win = sqrt(hanning(63)); win = [0 ; win(1:32)];
// fprintf(1, '\t%.14f, %.14f, %.14f,\n', win);
ALIGN16_BEG const float ALIGN16_END WebRtcAec_sqrtHanning[65] = {
    0.00000000000000f, 0.02454122852291f, 0.04906767432742f, 0.07356456359967f,
    0.09801714032956f, 0.12241067519922f, 0.14673047445536f, 0.17096188876030f,
    0.19509032201613f, 0.21910124015687f, 0.24298017990326f, 0.26671275747490f,
    0.29028467725446f, 0.31368174039889f, 0.33688985339222f, 0.35989503653499f,
    0.38268343236509f, 0.40524131400499f, 0.42755509343028f, 0.44961132965461f,
    0.47139673682600f, 0.49289819222978f, 0.51410274419322f, 0.53499761988710f,
    0.55557023301960f, 0.57580819141785f, 0.59569930449243f, 0.61523159058063f,
    0.63439328416365f, 0.65317284295378f, 0.67155895484702f, 0.68954054473707f,
    0.70710678118655f, 0.72424708295147f, 0.74095112535496f, 0.75720884650648f,
    0.77301045336274f, 0.78834642762661f, 0.80320753148064f, 0.81758481315158f,
    0.83146961230255f, 0.84485356524971f, 0.85772861000027f, 0.87008699110871f,
    0.88192126434835f, 0.89322430119552f, 0.90398929312344f, 0.91420975570353f,
    0.92387953251129f, 0.93299279883474f, 0.94154406518302f, 0.94952818059304f,
    0.95694033573221f, 0.96377606579544f, 0.97003125319454f, 0.97570213003853f,
    0.98078528040323f, 0.98527764238894f, 0.98917650996478f, 0.99247953459871f,
    0.99518472667220f, 0.99729045667869f, 0.99879545620517f, 0.99969881869620f,
    1.00000000000000f};

// Matlab code to produce table:
// weightCurve = [0 ; 0.3 * sqrt(linspace(0,1,64))' + 0.1];
// fprintf(1, '\t%.4f, %.4f, %.4f, %.4f, %.4f, %.4f,\n', weightCurve);
ALIGN16_BEG const float ALIGN16_END WebRtcAec_weightCurve[65] = {
    0.0000f, 0.1000f, 0.1378f, 0.1535f, 0.1655f, 0.1756f, 0.1845f, 0.1926f,
    0.2000f, 0.2069f, 0.2134f, 0.2195f, 0.2254f, 0.2309f, 0.2363f, 0.2414f,
    0.2464f, 0.2512f, 0.2558f, 0.2604f, 0.2648f, 0.2690f, 0.2732f, 0.2773f,
    0.2813f, 0.2852f, 0.2890f, 0.2927f, 0.2964f, 0.3000f, 0.3035f, 0.3070f,
    0.3104f, 0.3138f, 0.3171f, 0.3204f, 0.3236f, 0.3268f, 0.3299f, 0.3330f,
    0.3360f, 0.3390f, 0.3420f, 0.3449f, 0.3478f, 0.3507f, 0.3535f, 0.3563f,
    0.3591f, 0.3619f, 0.3646f, 0.3673f, 0.3699f, 0.3726f, 0.3752f, 0.3777f,
    0.3803f, 0.3828f, 0.3854f, 0.3878f, 0.3903f, 0.3928f, 0.3952f, 0.3976f,
    0.4000f};

// Matlab code to produce table:
// overDriveCurve = [sqrt(linspace(0,1,65))' + 1];
// fprintf(1, '\t%.4f, %.4f, %.4f, %.4f, %.4f, %.4f,\n', overDriveCurve);
ALIGN16_BEG const float ALIGN16_END WebRtcAec_overDriveCurve[65] = {
    1.0000f, 1.1250f, 1.1768f, 1.2165f, 1.2500f, 1.2795f, 1.3062f, 1.3307f,
    1.3536f, 1.3750f, 1.3953f, 1.4146f, 1.4330f, 1.4507f, 1.4677f, 1.4841f,
    1.5000f, 1.5154f, 1.5303f, 1.5449f, 1.5590f, 1.5728f, 1.5863f, 1.5995f,
    1.6124f, 1.6250f, 1.6374f, 1.6495f, 1.6614f, 1.6731f, 1.6847f, 1.6960f,
    1.7071f, 1.7181f, 1.7289f, 1.7395f, 1.7500f, 1.7603f, 1.7706f, 1.7806f,
    1.7906f, 1.8004f, 1.8101f, 1.8197f, 1.8292f, 1.8385f, 1.8478f, 1.8570f,
    1.8660f, 1.8750f, 1.8839f, 1.8927f, 1.9014f, 1.9100f, 1.9186f, 1.9270f,
    1.9354f, 1.9437f, 1.9520f, 1.9601f, 1.9682f, 1.9763f, 1.9843f, 1.9922f,
    2.0000f};

// Two sets of parameters, one for the extended filter mode.
const float WebRtcAec_kExtendedSmoothingCoefficients[2][2] = {{0.9f, 0.1f},
                                                              {0.92f, 0.08f}};
const float WebRtcAec_kNormalSmoothingCoefficients[2][2] = {{0.9f, 0.1f},
                                                            {0.93f, 0.07f}};

__inline static float MulRe(float aRe, float aIm, float bRe, float bIm) {
  return aRe * bRe - aIm * bIm;
}

__inline static float MulIm(float aRe, float aIm, float bRe, float bIm) {
  return aRe * bIm + aIm * bRe;
}

static void FilterFarC(int num_partitions,
                      int x_fft_buf_block_pos,
                      float x_fft_buf[2][kExtendedNumPartitions * PART_LEN1],
                      float h_fft_buf[2][kExtendedNumPartitions * PART_LEN1],
                      float y_fft[2][PART_LEN1]) {
  int i;
  for (i = 0; i < num_partitions; i++) {
    int j;
    int xPos = (i + x_fft_buf_block_pos) * PART_LEN1;
    int pos = i * PART_LEN1;
    // Check for wrap
    if (i + x_fft_buf_block_pos >= num_partitions) {
      xPos -= num_partitions * (PART_LEN1);
    }

    for (j = 0; j < PART_LEN1; j++) {
      y_fft[0][j] += MulRe(x_fft_buf[0][xPos + j], x_fft_buf[1][xPos + j],
                           h_fft_buf[0][pos + j], h_fft_buf[1][pos + j]);
      y_fft[1][j] += MulIm(x_fft_buf[0][xPos + j], x_fft_buf[1][xPos + j],
                           h_fft_buf[0][pos + j], h_fft_buf[1][pos + j]);
    }
  }
}

static void ScaleErrorSignalC(float mu,
                             float error_threshold,
                             float x_pow[PART_LEN1],
                             float ef[2][PART_LEN1]) {
  int i;
  float abs_ef;
  for (i = 0; i < (PART_LEN1); i++) {
    ef[0][i] /= (x_pow[i] + 1e-10f);
    ef[1][i] /= (x_pow[i] + 1e-10f);
    abs_ef = sqrtf(ef[0][i] * ef[0][i] + ef[1][i] * ef[1][i]);

    if (abs_ef > error_threshold) {
      abs_ef = error_threshold / (abs_ef + 1e-10f);
      ef[0][i] *= abs_ef;
      ef[1][i] *= abs_ef;
    }

    // Stepsize factor
    ef[0][i] *= mu;
    ef[1][i] *= mu;
  }
}

static void FilterAdaptationC(
    const OouraFft& ooura_fft,
    int num_partitions,
    int x_fft_buf_block_pos,
    float x_fft_buf[2][kExtendedNumPartitions * PART_LEN1],
    float e_fft[2][PART_LEN1],
    float h_fft_buf[2][kExtendedNumPartitions * PART_LEN1]) {
  int i, j;
  float fft[PART_LEN2];
  for (i = 0; i < num_partitions; i++) {
    int xPos = (i + x_fft_buf_block_pos) * (PART_LEN1);
    int pos;
    // Check for wrap
    if (i + x_fft_buf_block_pos >= num_partitions) {
      xPos -= num_partitions * PART_LEN1;
    }

    pos = i * PART_LEN1;

    for (j = 0; j < PART_LEN; j++) {
      fft[2 * j] = MulRe(x_fft_buf[0][xPos + j], -x_fft_buf[1][xPos + j],
                         e_fft[0][j], e_fft[1][j]);
      fft[2 * j + 1] = MulIm(x_fft_buf[0][xPos + j], -x_fft_buf[1][xPos + j],
                             e_fft[0][j], e_fft[1][j]);
    }
    fft[1] =
        MulRe(x_fft_buf[0][xPos + PART_LEN], -x_fft_buf[1][xPos + PART_LEN],
              e_fft[0][PART_LEN], e_fft[1][PART_LEN]);

    ooura_fft.InverseFft(fft);
    memset(fft + PART_LEN, 0, sizeof(float) * PART_LEN);

    // fft scaling
    {
      float scale = 2.0f / PART_LEN2;
      for (j = 0; j < PART_LEN; j++) {
        fft[j] *= scale;
      }
    }
    ooura_fft.Fft(fft);

    h_fft_buf[0][pos] += fft[0];
    h_fft_buf[0][pos + PART_LEN] += fft[1];

    for (j = 1; j < PART_LEN; j++) {
      h_fft_buf[0][pos + j] += fft[2 * j];
      h_fft_buf[1][pos + j] += fft[2 * j + 1];
    }
  }
}

static void OverdriveC(float overdrive_scaling,
                      const float hNlFb,
                      float hNl[PART_LEN1]) {
  for (int i = 0; i < PART_LEN1; ++i) {
    // Weight subbands
    if (hNl[i] > hNlFb) {
      hNl[i] = WebRtcAec_weightCurve[i] * hNlFb +
               (1 - WebRtcAec_weightCurve[i]) * hNl[i];
    }
    hNl[i] = powf(hNl[i], overdrive_scaling * WebRtcAec_overDriveCurve[i]);
  }
}

static void SuppressC(const float hNl[PART_LEN1], float efw[2][PART_LEN1]) {
  for (int i = 0; i < PART_LEN1; ++i) {
    // Suppress error signal
    efw[0][i] *= hNl[i];
    efw[1][i] *= hNl[i];

    // Ooura fft returns incorrect sign on imaginary component. It matters here
    // because we are making an additive change with comfort noise.
    efw[1][i] *= -1;
  }
}

static int PartitionDelayC(
    int num_partitions,
    float h_fft_buf[2][kExtendedNumPartitions * PART_LEN1]) {
  // Measures the energy in each filter partition and returns the partition with
  // highest energy.
  // TODO(bjornv): Spread computational cost by computing one partition per
  // block?
  float wfEnMax = 0;
  int i;
  int delay = 0;

  for (i = 0; i < num_partitions; i++) {
    int j;
    int pos = i * PART_LEN1;
    float wfEn = 0;
    for (j = 0; j < PART_LEN1; j++) {
      wfEn += h_fft_buf[0][pos + j] * h_fft_buf[0][pos + j] +
              h_fft_buf[1][pos + j] * h_fft_buf[1][pos + j];
    }

    if (wfEn > wfEnMax) {
      wfEnMax = wfEn;
      delay = i;
    }
  }
  return delay;
}

static void ComputeCoherenceC(const CoherenceState* coherence_state,
                             float* cohde,
                             float* cohxd) {
  // Subband coherence
  for (int i = 0; i < PART_LEN1; i++) {
    cohde[i] = (coherence_state->sde[i][0] * coherence_state->sde[i][0] +
                coherence_state->sde[i][1] * coherence_state->sde[i][1]) /
               (coherence_state->sd[i] * coherence_state->se[i] + 1e-10f);
    cohxd[i] = (coherence_state->sxd[i][0] * coherence_state->sxd[i][0] +
                coherence_state->sxd[i][1] * coherence_state->sxd[i][1]) /
               (coherence_state->sx[i] * coherence_state->sd[i] + 1e-10f);
  }
}

// Threshold to protect against the ill-effects of a zero far-end.
const float WebRtcAec_kMinFarendPSD = 15;

// Updates the following smoothed Power Spectral Densities (PSD):
//  - sd  : near-end
//  - se  : residual echo
//  - sx  : far-end
//  - sde : cross-PSD of near-end and residual echo
//  - sxd : cross-PSD of near-end and far-end
//
// In addition to updating the PSDs, also the filter diverge state is
// determined.
static void UpdateCoherenceSpectraC(int mult,
                                   bool extended_filter_enabled,
                                   float efw[2][PART_LEN1],
                                   float dfw[2][PART_LEN1],
                                   float xfw[2][PART_LEN1],
                                   CoherenceState* coherence_state,
                                   short* filter_divergence_state,
                                   int* extreme_filter_divergence) {
  // Power estimate smoothing coefficients.
  const float* ptrGCoh =
      extended_filter_enabled
          ? WebRtcAec_kExtendedSmoothingCoefficients[mult - 1]
          : WebRtcAec_kNormalSmoothingCoefficients[mult - 1];
  int i;
  float sdSum = 0, seSum = 0;

  for (i = 0; i < PART_LEN1; i++) {
    coherence_state->sd[i] =
        ptrGCoh[0] * coherence_state->sd[i] +
        ptrGCoh[1] * (dfw[0][i] * dfw[0][i] + dfw[1][i] * dfw[1][i]);
    coherence_state->se[i] =
        ptrGCoh[0] * coherence_state->se[i] +
        ptrGCoh[1] * (efw[0][i] * efw[0][i] + efw[1][i] * efw[1][i]);
    // We threshold here to protect against the ill-effects of a zero farend.
    // The threshold is not arbitrarily chosen, but balances protection and
    // adverse interaction with the algorithm's tuning.
    // TODO(bjornv): investigate further why this is so sensitive.
    coherence_state->sx[i] =
        ptrGCoh[0] * coherence_state->sx[i] +
        ptrGCoh[1] *
            WEBRTC_SPL_MAX(xfw[0][i] * xfw[0][i] + xfw[1][i] * xfw[1][i],
                           WebRtcAec_kMinFarendPSD);

    coherence_state->sde[i][0] =
        ptrGCoh[0] * coherence_state->sde[i][0] +
        ptrGCoh[1] * (dfw[0][i] * efw[0][i] + dfw[1][i] * efw[1][i]);
    coherence_state->sde[i][1] =
        ptrGCoh[0] * coherence_state->sde[i][1] +
        ptrGCoh[1] * (dfw[0][i] * efw[1][i] - dfw[1][i] * efw[0][i]);

    coherence_state->sxd[i][0] =
        ptrGCoh[0] * coherence_state->sxd[i][0] +
        ptrGCoh[1] * (dfw[0][i] * xfw[0][i] + dfw[1][i] * xfw[1][i]);
    coherence_state->sxd[i][1] =
        ptrGCoh[0] * coherence_state->sxd[i][1] +
        ptrGCoh[1] * (dfw[0][i] * xfw[1][i] - dfw[1][i] * xfw[0][i]);

    sdSum += coherence_state->sd[i];
    seSum += coherence_state->se[i];
  }

  // Divergent filter safeguard update.
  *filter_divergence_state =
      (*filter_divergence_state ? 1.05f : 1.0f) * seSum > sdSum;

  // Signal extreme filter divergence if the error is significantly larger
  // than the nearend (13 dB).
  *extreme_filter_divergence = (seSum > (19.95f * sdSum));
}

// Window time domain data to be used by the fft.
__inline static void WindowDataC(float* x_windowed, const float* x) {
  int i;
  for (i = 0; i < PART_LEN; i++) {
    x_windowed[i] = x[i] * WebRtcAec_sqrtHanning[i];
    x_windowed[PART_LEN + i] =
        x[PART_LEN + i] * WebRtcAec_sqrtHanning[PART_LEN - i];
  }
}

// Puts fft output data into a complex valued array.
__inline static void StoreAsComplexC(const float* data,
                                    float data_complex[2][PART_LEN1]) {
  int i;
  data_complex[0][0] = data[0];
  data_complex[1][0] = 0;
  for (i = 1; i < PART_LEN; i++) {
    data_complex[0][i] = data[2 * i];
    data_complex[1][i] = data[2 * i + 1];
  }
  data_complex[0][PART_LEN] = data[1];
  data_complex[1][PART_LEN] = 0;
}

void WebRtcAec_InitAec_c(void) {
  WebRtcAec_FilterFar = FilterFarC;
  WebRtcAec_ScaleErrorSignal = ScaleErrorSignalC;
  WebRtcAec_FilterAdaptation = FilterAdaptationC;
  WebRtcAec_Overdrive = OverdriveC;
  WebRtcAec_Suppress = SuppressC;
  WebRtcAec_ComputeCoherence = ComputeCoherenceC;
  WebRtcAec_UpdateCoherenceSpectra = UpdateCoherenceSpectraC;
  WebRtcAec_StoreAsComplex = StoreAsComplexC;
  WebRtcAec_PartitionDelay = PartitionDelayC;
  WebRtcAec_WindowData = WindowDataC;
}
// }  // namespace webrtc

#endif
