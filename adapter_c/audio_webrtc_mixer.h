
#ifndef _AUDIO_WEBRTC_MIXER_H_
#define _AUDIO_WEBRTC_MIXER_H_

#include "dsp_common.h"

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct AUDIO_WEBRTC_MIXER AUDIO_WEBRTC_MIXER_S;
typedef struct AUDIO_WEBRTC_MIXER_PARAM
{
    DSP_S32 s32Channels;
    DSP_S32 s32Bitwidth;
    DSP_S32 s32Samplerate;
} AUDIO_WEBRTC_MIXER_PARAM_S;

DSP_S32 Audio_Webrtc_CreateMixGrp(AUDIO_WEBRTC_MIXER_S **ppMixer, AUDIO_WEBRTC_MIXER_PARAM_S *pstParam);
DSP_S32 Audio_Webrtc_DestroyMixGrp(AUDIO_WEBRTC_MIXER_S *pMixer);
DSP_S32 Audio_Webrtc_CreateMixChn(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_S32 *s32Id, AUDIO_WEBRTC_MIXER_PARAM_S *pstParam);
DSP_S32 Audio_Webrtc_DestroyMixChn(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_S32 s32Id);
DSP_S32 Audio_Webrtc_SentMixChnFrame(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_S32 s32Id, DSP_PUB_DATAINFO_S *pstData);
DSP_S32 Audio_Webrtc_GetMixGrpFrame(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_PUB_DATAINFO_S *pstData);
DSP_S32 Audio_Webrtc_ReleaseMixGrpFrame(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_PUB_DATAINFO_S *pstData);

#if defined(__cplusplus)
}
#endif

#endif /* AUDIO_WEBRTC_MIXER_H */
