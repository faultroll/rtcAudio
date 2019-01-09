/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/checks.h"
#include "modules/audio_processing/ns/noise_suppression_x.h"
#include "modules/audio_processing/ns/nsx_core.h"
#include "modules/audio_processing/ns/nsx_defines.h"

extern const int16_t WebRtcNsx_kLogTable[9];
extern const int16_t WebRtcNsx_kCounterDiv[201];
extern const int16_t WebRtcNsx_kLogTableFrac[256];

static const int16_t kIndicatorTable[17] = {
  0, 2017, 3809, 5227, 6258, 6963, 7424, 7718,
  7901, 8014, 8084, 8126, 8152, 8168, 8177, 8183, 8187
};

// Compute speech/noise probability
// speech/noise probability is returned in: probSpeechFinal
//snrLocPrior is the prior SNR for each frequency (in Q11)
//snrLocPost is the post SNR for each frequency (in Q11)
void WebRtcNsx_SpeechNoiseProbC(NoiseSuppressionFixedC* inst,
                                uint16_t* nonSpeechProbFinal,
                                uint32_t* priorLocSnr,
                                uint32_t* postLocSnr) {
  uint32_t zeros, num, den, tmpU32no1, tmpU32no2, tmpU32no3;
  int32_t invLrtFX, indPriorFX, tmp32, tmp32no1, tmp32no2, besselTmpFX32;
  int32_t frac32, logTmp;
  int32_t logLrtTimeAvgKsumFX;
  int16_t indPriorFX16;
  int16_t tmp16, tmp16no1, tmp16no2, tmpIndFX, tableIndex, frac, intPart;
  size_t i;
  int normTmp, normTmp2, nShifts;

  // compute feature based on average LR factor
  // this is the average over all frequencies of the smooth log LRT
  logLrtTimeAvgKsumFX = 0;
  for (i = 0; i < inst->magnLen; i++) {
    besselTmpFX32 = (int32_t)postLocSnr[i]; // Q11
    normTmp = WebRtcSpl_NormU32(postLocSnr[i]);
    num = postLocSnr[i] << normTmp;  // Q(11+normTmp)
    if (normTmp > 10) {
      den = priorLocSnr[i] << (normTmp - 11);  // Q(normTmp)
    } else {
      den = priorLocSnr[i] >> (11 - normTmp);  // Q(normTmp)
    }
    if (den > 0) {
      besselTmpFX32 -= num / den;  // Q11
    } else {
      besselTmpFX32 = 0;
    }

    // inst->logLrtTimeAvg[i] += LRT_TAVG * (besselTmp - log(snrLocPrior)
    //                                       - inst->logLrtTimeAvg[i]);
    // Here, LRT_TAVG = 0.5
    zeros = WebRtcSpl_NormU32(priorLocSnr[i]);
    frac32 = (int32_t)(((priorLocSnr[i] << zeros) & 0x7FFFFFFF) >> 19);
    tmp32 = (frac32 * frac32 * -43) >> 19;
    tmp32 += ((int16_t)frac32 * 5412) >> 12;
    frac32 = tmp32 + 37;
    // tmp32 = log2(priorLocSnr[i])
    tmp32 = (int32_t)(((31 - zeros) << 12) + frac32) - (11 << 12); // Q12
    logTmp = (tmp32 * 178) >> 8;  // log2(priorLocSnr[i])*log(2)
    // tmp32no1 = LRT_TAVG * (log(snrLocPrior) + inst->logLrtTimeAvg[i]) in Q12.
    tmp32no1 = (logTmp + inst->logLrtTimeAvgW32[i]) / 2;
    inst->logLrtTimeAvgW32[i] += (besselTmpFX32 - tmp32no1); // Q12

    logLrtTimeAvgKsumFX += inst->logLrtTimeAvgW32[i]; // Q12
  }
  inst->featureLogLrt = (logLrtTimeAvgKsumFX * BIN_SIZE_LRT) >>
      (inst->stages + 11);

  // done with computation of LR factor

  //
  //compute the indicator functions
  //

  // average LRT feature
  // FLOAT code
  // indicator0 = 0.5 * (tanh(widthPrior *
  //                      (logLrtTimeAvgKsum - threshPrior0)) + 1.0);
  tmpIndFX = 16384; // Q14(1.0)
  tmp32no1 = logLrtTimeAvgKsumFX - inst->thresholdLogLrt; // Q12
  nShifts = 7 - inst->stages; // WIDTH_PR_MAP_SHIFT - inst->stages + 5;
  //use larger width in tanh map for pause regions
  if (tmp32no1 < 0) {
    tmpIndFX = 0;
    tmp32no1 = -tmp32no1;
    //widthPrior = widthPrior * 2.0;
    nShifts++;
  }
  tmp32no1 = WEBRTC_SPL_SHIFT_W32(tmp32no1, nShifts); // Q14
  // compute indicator function: sigmoid map
  if (tmp32no1 < (16 << 14) && tmp32no1 >= 0) {
    tableIndex = (int16_t)(tmp32no1 >> 14);
    tmp16no2 = kIndicatorTable[tableIndex];
    tmp16no1 = kIndicatorTable[tableIndex + 1] - kIndicatorTable[tableIndex];
    frac = (int16_t)(tmp32no1 & 0x00003fff); // Q14
    tmp16no2 += (int16_t)((tmp16no1 * frac) >> 14);
    if (tmpIndFX == 0) {
      tmpIndFX = 8192 - tmp16no2; // Q14
    } else {
      tmpIndFX = 8192 + tmp16no2; // Q14
    }
  }
  indPriorFX = inst->weightLogLrt * tmpIndFX;  // 6*Q14

  //spectral flatness feature
  if (inst->weightSpecFlat) {
    tmpU32no1 = WEBRTC_SPL_UMUL(inst->featureSpecFlat, 400); // Q10
    tmpIndFX = 16384; // Q14(1.0)
    //use larger width in tanh map for pause regions
    tmpU32no2 = inst->thresholdSpecFlat - tmpU32no1; //Q10
    nShifts = 4;
    if (inst->thresholdSpecFlat < tmpU32no1) {
      tmpIndFX = 0;
      tmpU32no2 = tmpU32no1 - inst->thresholdSpecFlat;
      //widthPrior = widthPrior * 2.0;
      nShifts++;
    }
    tmpU32no1 = WebRtcSpl_DivU32U16(tmpU32no2 << nShifts, 25);  // Q14
    // compute indicator function: sigmoid map
    // FLOAT code
    // indicator1 = 0.5 * (tanh(sgnMap * widthPrior *
    //                          (threshPrior1 - tmpFloat1)) + 1.0);
    if (tmpU32no1 < (16 << 14)) {
      tableIndex = (int16_t)(tmpU32no1 >> 14);
      tmp16no2 = kIndicatorTable[tableIndex];
      tmp16no1 = kIndicatorTable[tableIndex + 1] - kIndicatorTable[tableIndex];
      frac = (int16_t)(tmpU32no1 & 0x00003fff); // Q14
      tmp16no2 += (int16_t)((tmp16no1 * frac) >> 14);
      if (tmpIndFX) {
        tmpIndFX = 8192 + tmp16no2; // Q14
      } else {
        tmpIndFX = 8192 - tmp16no2; // Q14
      }
    }
    indPriorFX += inst->weightSpecFlat * tmpIndFX;  // 6*Q14
  }

  //for template spectral-difference
  if (inst->weightSpecDiff) {
    tmpU32no1 = 0;
    if (inst->featureSpecDiff) {
      normTmp = WEBRTC_SPL_MIN(20 - inst->stages,
                               WebRtcSpl_NormU32(inst->featureSpecDiff));
      RTC_DCHECK_GE(normTmp, 0);
      tmpU32no1 = inst->featureSpecDiff << normTmp;  // Q(normTmp-2*stages)
      tmpU32no2 = inst->timeAvgMagnEnergy >> (20 - inst->stages - normTmp);
      if (tmpU32no2 > 0) {
        // Q(20 - inst->stages)
        tmpU32no1 /= tmpU32no2;
      } else {
        tmpU32no1 = (uint32_t)(0x7fffffff);
      }
    }
    tmpU32no3 = (inst->thresholdSpecDiff << 17) / 25;
    tmpU32no2 = tmpU32no1 - tmpU32no3;
    nShifts = 1;
    tmpIndFX = 16384; // Q14(1.0)
    //use larger width in tanh map for pause regions
    if (tmpU32no2 & 0x80000000) {
      tmpIndFX = 0;
      tmpU32no2 = tmpU32no3 - tmpU32no1;
      //widthPrior = widthPrior * 2.0;
      nShifts--;
    }
    tmpU32no1 = tmpU32no2 >> nShifts;
    // compute indicator function: sigmoid map
    /* FLOAT code
     indicator2 = 0.5 * (tanh(widthPrior * (tmpFloat1 - threshPrior2)) + 1.0);
     */
    if (tmpU32no1 < (16 << 14)) {
      tableIndex = (int16_t)(tmpU32no1 >> 14);
      tmp16no2 = kIndicatorTable[tableIndex];
      tmp16no1 = kIndicatorTable[tableIndex + 1] - kIndicatorTable[tableIndex];
      frac = (int16_t)(tmpU32no1 & 0x00003fff); // Q14
      tmp16no2 += (int16_t)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
                    tmp16no1, frac, 14);
      if (tmpIndFX) {
        tmpIndFX = 8192 + tmp16no2;
      } else {
        tmpIndFX = 8192 - tmp16no2;
      }
    }
    indPriorFX += inst->weightSpecDiff * tmpIndFX;  // 6*Q14
  }

  //combine the indicator function with the feature weights
  // FLOAT code
  // indPrior = 1 - (weightIndPrior0 * indicator0 + weightIndPrior1 *
  //                 indicator1 + weightIndPrior2 * indicator2);
  indPriorFX16 = WebRtcSpl_DivW32W16ResW16(98307 - indPriorFX, 6); // Q14
  // done with computing indicator function

  //compute the prior probability
  // FLOAT code
  // inst->priorNonSpeechProb += PRIOR_UPDATE *
  //                             (indPriorNonSpeech - inst->priorNonSpeechProb);
  tmp16 = indPriorFX16 - inst->priorNonSpeechProb; // Q14
  inst->priorNonSpeechProb += (int16_t)((PRIOR_UPDATE_Q14 * tmp16) >> 14);

  //final speech probability: combine prior model with LR factor:

  memset(nonSpeechProbFinal, 0, sizeof(uint16_t) * inst->magnLen);

  if (inst->priorNonSpeechProb > 0) {
    for (i = 0; i < inst->magnLen; i++) {
      // FLOAT code
      // invLrt = exp(inst->logLrtTimeAvg[i]);
      // invLrt = inst->priorSpeechProb * invLrt;
      // nonSpeechProbFinal[i] = (1.0 - inst->priorSpeechProb) /
      //                         (1.0 - inst->priorSpeechProb + invLrt);
      // invLrt = (1.0 - inst->priorNonSpeechProb) * invLrt;
      // nonSpeechProbFinal[i] = inst->priorNonSpeechProb /
      //                         (inst->priorNonSpeechProb + invLrt);
      if (inst->logLrtTimeAvgW32[i] < 65300) {
        tmp32no1 = (inst->logLrtTimeAvgW32[i] * 23637) >> 14;  // Q12
        intPart = (int16_t)(tmp32no1 >> 12);
        if (intPart < -8) {
          intPart = -8;
        }
        frac = (int16_t)(tmp32no1 & 0x00000fff); // Q12

        // Quadratic approximation of 2^frac
        tmp32no2 = (frac * frac * 44) >> 19;  // Q12.
        tmp32no2 += (frac * 84) >> 7;  // Q12
        invLrtFX = (1 << (8 + intPart)) +
            WEBRTC_SPL_SHIFT_W32(tmp32no2, intPart - 4); // Q8

        normTmp = WebRtcSpl_NormW32(invLrtFX);
        normTmp2 = WebRtcSpl_NormW16((16384 - inst->priorNonSpeechProb));
        if (normTmp + normTmp2 >= 7) {
          if (normTmp + normTmp2 < 15) {
            invLrtFX >>= 15 - normTmp2 - normTmp;
            // Q(normTmp+normTmp2-7)
            tmp32no1 = invLrtFX * (16384 - inst->priorNonSpeechProb);
            // Q(normTmp+normTmp2+7)
            invLrtFX = WEBRTC_SPL_SHIFT_W32(tmp32no1, 7 - normTmp - normTmp2);
                                                                  // Q14
          } else {
            tmp32no1 = invLrtFX * (16384 - inst->priorNonSpeechProb);
                                                                  // Q22
            invLrtFX = tmp32no1 >> 8;  // Q14.
          }

          tmp32no1 = (int32_t)inst->priorNonSpeechProb << 8;  // Q22

          nonSpeechProbFinal[i] = tmp32no1 /
              (inst->priorNonSpeechProb + invLrtFX);  // Q8
        }
      }
    }
  }
}

// Update the noise estimation information.
static void UpdateNoiseEstimate(NoiseSuppressionFixedC* inst, int offset) {
  int32_t tmp32no1 = 0;
  int32_t tmp32no2 = 0;
  int16_t tmp16 = 0;
  const int16_t kExp2Const = 11819; // Q13

  size_t i = 0;

  tmp16 = WebRtcSpl_MaxValueW16(inst->noiseEstLogQuantile + offset,
                                   inst->magnLen);
  // Guarantee a Q-domain as high as possible and still fit in int16
  inst->qNoise = 14 - (int) WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
                   kExp2Const, tmp16, 21);
  for (i = 0; i < inst->magnLen; i++) {
    // inst->quantile[i]=exp(inst->lquantile[offset+i]);
    // in Q21
    tmp32no2 = kExp2Const * inst->noiseEstLogQuantile[offset + i];
    tmp32no1 = (0x00200000 | (tmp32no2 & 0x001FFFFF)); // 2^21 + frac
    tmp16 = (int16_t)(tmp32no2 >> 21);
    tmp16 -= 21;// shift 21 to get result in Q0
    tmp16 += (int16_t) inst->qNoise; //shift to get result in Q(qNoise)
    if (tmp16 < 0) {
      tmp32no1 >>= -tmp16;
    } else {
      tmp32no1 <<= tmp16;
    }
    inst->noiseEstQuantile[i] = WebRtcSpl_SatW32ToW16(tmp32no1);
  }
}

// Noise Estimation
void WebRtcNsx_NoiseEstimationC(NoiseSuppressionFixedC* inst,
                                uint16_t* magn,
                                uint32_t* noise,
                                int16_t* q_noise) {
  int16_t lmagn[HALF_ANAL_BLOCKL], counter, countDiv;
  int16_t countProd, delta, zeros, frac;
  int16_t log2, tabind, logval, tmp16, tmp16no1, tmp16no2;
  const int16_t log2_const = 22713; // Q15
  const int16_t width_factor = 21845;

  size_t i, s, offset;

  tabind = inst->stages - inst->normData;
  RTC_DCHECK_LT(tabind, 9);
  RTC_DCHECK_GT(tabind, -9);
  if (tabind < 0) {
    logval = -WebRtcNsx_kLogTable[-tabind];
  } else {
    logval = WebRtcNsx_kLogTable[tabind];
  }

  // lmagn(i)=log(magn(i))=log(2)*log2(magn(i))
  // magn is in Q(-stages), and the real lmagn values are:
  // real_lmagn(i)=log(magn(i)*2^stages)=log(magn(i))+log(2^stages)
  // lmagn in Q8
  for (i = 0; i < inst->magnLen; i++) {
    if (magn[i]) {
      zeros = WebRtcSpl_NormU32((uint32_t)magn[i]);
      frac = (int16_t)((((uint32_t)magn[i] << zeros)
                              & 0x7FFFFFFF) >> 23);
      // log2(magn(i))
      RTC_DCHECK_LT(frac, 256);
      log2 = (int16_t)(((31 - zeros) << 8)
                             + WebRtcNsx_kLogTableFrac[frac]);
      // log2(magn(i))*log(2)
      lmagn[i] = (int16_t)((log2 * log2_const) >> 15);
      // + log(2^stages)
      lmagn[i] += logval;
    } else {
      lmagn[i] = logval;//0;
    }
  }

  // loop over simultaneous estimates
  for (s = 0; s < SIMULT; s++) {
    offset = s * inst->magnLen;

    // Get counter values from state
    counter = inst->noiseEstCounter[s];
    RTC_DCHECK_LT(counter, 201);
    countDiv = WebRtcNsx_kCounterDiv[counter];
    countProd = (int16_t)(counter * countDiv);

    // quant_est(...)
    for (i = 0; i < inst->magnLen; i++) {
      // compute delta
      if (inst->noiseEstDensity[offset + i] > 512) {
        // Get the value for delta by shifting intead of dividing.
        int factor = WebRtcSpl_NormW16(inst->noiseEstDensity[offset + i]);
        delta = (int16_t)(FACTOR_Q16 >> (14 - factor));
      } else {
        delta = FACTOR_Q7;
        if (inst->blockIndex < END_STARTUP_LONG) {
          // Smaller step size during startup. This prevents from using
          // unrealistic values causing overflow.
          delta = FACTOR_Q7_STARTUP;
        }
      }

      // update log quantile estimate
      tmp16 = (int16_t)((delta * countDiv) >> 14);
      if (lmagn[i] > inst->noiseEstLogQuantile[offset + i]) {
        // +=QUANTILE*delta/(inst->counter[s]+1) QUANTILE=0.25, =1 in Q2
        // CounterDiv=1/(inst->counter[s]+1) in Q15
        tmp16 += 2;
        inst->noiseEstLogQuantile[offset + i] += tmp16 / 4;
      } else {
        tmp16 += 1;
        // *(1-QUANTILE), in Q2 QUANTILE=0.25, 1-0.25=0.75=3 in Q2
        // TODO(bjornv): investigate why we need to truncate twice.
        tmp16no2 = (int16_t)((tmp16 / 2) * 3 / 2);
        inst->noiseEstLogQuantile[offset + i] -= tmp16no2;
        if (inst->noiseEstLogQuantile[offset + i] < logval) {
          // This is the smallest fixed point representation we can
          // have, hence we limit the output.
          inst->noiseEstLogQuantile[offset + i] = logval;
        }
      }

      // update density estimate
      if (WEBRTC_SPL_ABS_W16(lmagn[i] - inst->noiseEstLogQuantile[offset + i])
          < WIDTH_Q8) {
        tmp16no1 = (int16_t)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
                     inst->noiseEstDensity[offset + i], countProd, 15);
        tmp16no2 = (int16_t)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
                     width_factor, countDiv, 15);
        inst->noiseEstDensity[offset + i] = tmp16no1 + tmp16no2;
      }
    }  // end loop over magnitude spectrum

    if (counter >= END_STARTUP_LONG) {
      inst->noiseEstCounter[s] = 0;
      if (inst->blockIndex >= END_STARTUP_LONG) {
        UpdateNoiseEstimate(inst, offset);
      }
    }
    inst->noiseEstCounter[s]++;

  }  // end loop over simultaneous estimates

  // Sequentially update the noise during startup
  if (inst->blockIndex < END_STARTUP_LONG) {
    UpdateNoiseEstimate(inst, offset);
  }

  for (i = 0; i < inst->magnLen; i++) {
    noise[i] = (uint32_t)(inst->noiseEstQuantile[i]); // Q(qNoise)
  }
  (*q_noise) = (int16_t)inst->qNoise;
}

// Filter the data in the frequency domain, and create spectrum.
void WebRtcNsx_PrepareSpectrumC(NoiseSuppressionFixedC* inst,
                                int16_t* freq_buf) {
  size_t i = 0, j = 0;

  for (i = 0; i < inst->magnLen; i++) {
    inst->real[i] = (int16_t)((inst->real[i] *
        (int16_t)(inst->noiseSupFilter[i])) >> 14);  // Q(normData-stages)
    inst->imag[i] = (int16_t)((inst->imag[i] *
        (int16_t)(inst->noiseSupFilter[i])) >> 14);  // Q(normData-stages)
  }

  freq_buf[0] = inst->real[0];
  freq_buf[1] = -inst->imag[0];
  for (i = 1, j = 2; i < inst->anaLen2; i += 1, j += 2) {
    freq_buf[j] = inst->real[i];
    freq_buf[j + 1] = -inst->imag[i];
  }
  freq_buf[inst->anaLen] = inst->real[inst->anaLen2];
  freq_buf[inst->anaLen + 1] = -inst->imag[inst->anaLen2];
}

// Denormalize the real-valued signal |in|, the output from inverse FFT.
void WebRtcNsx_DenormalizeC(NoiseSuppressionFixedC* inst,
                            int16_t* in,
                            int factor) {
  size_t i = 0;
  int32_t tmp32 = 0;
  for (i = 0; i < inst->anaLen; i += 1) {
    tmp32 = WEBRTC_SPL_SHIFT_W32((int32_t)in[i],
                                 factor - inst->normData);
    inst->real[i] = WebRtcSpl_SatW32ToW16(tmp32); // Q0
  }
}

// For the noise supression process, synthesis, read out fully processed
// segment, and update synthesis buffer.
void WebRtcNsx_SynthesisUpdateC(NoiseSuppressionFixedC* inst,
                                int16_t* out_frame,
                                int16_t gain_factor) {
  size_t i = 0;
  int16_t tmp16a = 0;
  int16_t tmp16b = 0;
  int32_t tmp32 = 0;

  // synthesis
  for (i = 0; i < inst->anaLen; i++) {
    tmp16a = (int16_t)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
                 inst->window[i], inst->real[i], 14); // Q0, window in Q14
    tmp32 = WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(tmp16a, gain_factor, 13); // Q0
    // Down shift with rounding
    tmp16b = WebRtcSpl_SatW32ToW16(tmp32); // Q0
    inst->synthesisBuffer[i] = WebRtcSpl_AddSatW16(inst->synthesisBuffer[i],
                                                   tmp16b); // Q0
  }

  // read out fully processed segment
  for (i = 0; i < inst->blockLen10ms; i++) {
    out_frame[i] = inst->synthesisBuffer[i]; // Q0
  }

  // update synthesis buffer
  memcpy(inst->synthesisBuffer, inst->synthesisBuffer + inst->blockLen10ms,
      (inst->anaLen - inst->blockLen10ms) * sizeof(*inst->synthesisBuffer));
  WebRtcSpl_ZerosArrayW16(inst->synthesisBuffer
      + inst->anaLen - inst->blockLen10ms, inst->blockLen10ms);
}

// Update analysis buffer for lower band, and window data before FFT.
void WebRtcNsx_AnalysisUpdateC(NoiseSuppressionFixedC* inst,
                               int16_t* out,
                               int16_t* new_speech) {
  size_t i = 0;

  // For lower band update analysis buffer.
  memcpy(inst->analysisBuffer, inst->analysisBuffer + inst->blockLen10ms,
      (inst->anaLen - inst->blockLen10ms) * sizeof(*inst->analysisBuffer));
  memcpy(inst->analysisBuffer + inst->anaLen - inst->blockLen10ms, new_speech,
      inst->blockLen10ms * sizeof(*inst->analysisBuffer));

  // Window data before FFT.
  for (i = 0; i < inst->anaLen; i++) {
    out[i] = (int16_t)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
               inst->window[i], inst->analysisBuffer[i], 14); // Q0
  }
}

// Normalize the real-valued signal |in|, the input to forward FFT.
void WebRtcNsx_NormalizeRealBufferC(NoiseSuppressionFixedC* inst,
                                    const int16_t* in,
                                    int16_t* out) {
  size_t i = 0;
  RTC_DCHECK_GE(inst->normData, 0);
  for (i = 0; i < inst->anaLen; ++i) {
    out[i] = in[i] << inst->normData;  // Q(normData)
  }
}
