
#ifndef _AUDIO_WEBRTC_MULTI_H_
#define _AUDIO_WEBRTC_MULTI_H_

#include "dsp_common.h"

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct AUDIO_WEBRTC_MULTI AUDIO_WEBRTC_MULTI_S;
typedef struct AUDIO_WEBRTC_MULTI_PARAM
{
    DSP_S32 s32Channels;
    DSP_S32 s32Bitwidth;
    DSP_S32 s32Samplerate;
} AUDIO_WEBRTC_MULTI_PARAM_S;

DSP_S32 Audio_Webrtc_CreateMulGrp(AUDIO_WEBRTC_MULTI_S **ppMulti, AUDIO_WEBRTC_MULTI_PARAM_S *pstParam);
DSP_S32 Audio_Webrtc_DestroyMulGrp(AUDIO_WEBRTC_MULTI_S *pMulti);
DSP_S32 Audio_Webrtc_CreateMulChn(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_S32 *s32Id, AUDIO_WEBRTC_MULTI_PARAM_S *pstParam);
DSP_S32 Audio_Webrtc_DestroyMulChn(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_S32 s32Id);
DSP_S32 Audio_Webrtc_SentMulGrpFrame(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_PUB_DATAINFO_S *pstData);
DSP_S32 Audio_Webrtc_GetMulChnFrame(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_S32 s32Id, DSP_PUB_DATAINFO_S *pstData);
DSP_S32 Audio_Webrtc_ReleaseMulChnFrame(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_S32 s32Id, DSP_PUB_DATAINFO_S *pstData);

#if defined(__cplusplus)
}
#endif

#endif /* _AUDIO_WEBRTC_MULTI_H_ */
