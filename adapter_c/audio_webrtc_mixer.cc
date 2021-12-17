
#include "audio_webrtc_mixer.h"
#include <stdio.h>
#include <string.h>
#include "webrtc/modules/audio_conference_mixer/include/audio_conference_mixer.h"
#include "rtc_base/atomic_ops.h"
#include "rtc_base/time_utils.h"
/* #include "webrtc/common_audio/resampler/include/resampler.h" */
// #include "webrtc/common_audio/ring_buffer.h"
#include "audio_webrtc_buffer.h"
#include "audio_webrtc_proc.h"

/* magic number 10 is webrtc::AudioConferenceMixerImpl::kProcessPeriodicityInMs */

class AudioCallback : public webrtc::AudioMixerOutputReceiver
{
public:
    virtual void NewMixedAudio(const int32_t id,
                               const webrtc::AudioFrame &generalAudioFrame,
                               const webrtc::AudioFrame **uniqueAudioFrames,
                               const uint32_t size)
    {
        // printf("callback Len=%d\n", size);

        m_audioFrame.CopyFrom(generalAudioFrame);
    }
public:
    webrtc::AudioFrame m_audioFrame;
};

class Participant : public webrtc::MixerParticipant
{
public:
    Participant(int id, int samplerate, int channels) :
        m_id(id),
        m_samplerate(samplerate),
        m_channels(channels)
    {
        // fix bugs that no audio input but gets noise
        m_audioFrame.Reset();
    }

    virtual int32_t GetAudioFrame(int32_t id, webrtc::AudioFrame *audioFrame)
    {
        // printf("GetAudioFrame id=%d\n", id);

        audioFrame->CopyFrom(m_audioFrame);

        return 0;
    }

    virtual int32_t NeededFrequency(int32_t id) const
    {
        return m_samplerate;
    }

    void filldata(const int16_t *pData, size_t iSamples)
    {
        m_audioFrame.id_ = m_id;
        m_audioFrame.num_channels_ = m_channels;
        m_audioFrame.samples_per_channel_ = m_samplerate / (rtc::kNumMillisecsPerSec / 10);
        m_audioFrame.sample_rate_hz_ = m_samplerate;
        m_audioFrame.speech_type_ = webrtc::AudioFrame::kNormalSpeech;
        m_audioFrame.vad_activity_ = webrtc::AudioFrame::kVadActive;

        memmove(m_audioFrame.mutable_data(), pData, SAMPLE2LENGTH(iSamples));
    }

public:
    webrtc::AudioFrame m_audioFrame;
    int m_id;
    int m_samplerate;
    int m_channels;
};

#define MIXER_USING_RESAMPLER
#define MIXER_PAR_NUM       (4)
#define MIXER_PAR_BUFDEPTH  (20)
struct AUDIO_WEBRTC_MIXER
{
    // int iId;
    AUDIO_WEBRTC_MIXER_PARAM_S stParam;
    int iSz10ms;
    void *pData;
    webrtc::AudioConferenceMixer *pMixerImpl;
    AudioCallback *pCb;

    struct
    {
        bool bEnable;
        int64_t i64PtsErr;
        AUDIO_WEBRTC_MIXER_PARAM_S stParam;
#if defined(MIXER_USING_RESAMPLER)
        // resampler
        struct
        {
            uint32_t u32Seq;
            uint64_t u64PtsOld;
            int iSz30ms;
            void *pData;
            /* webrtc::Resampler *pResampler; */
            AudioWebrtcProc *pAproc;
        };
#endif
        // RingBuffer *pInput; // ringbuffer, if get failed, then RemoveSource in this Mix
        uint32_t u32Size; // buffer size
        AudioWebrtcBuffer *pInput;
        Participant *pPar;
    } stParInfo[MIXER_PAR_NUM];
};

DSP_S32 Audio_Webrtc_CreateMixGrp(AUDIO_WEBRTC_MIXER_S **ppMixer, AUDIO_WEBRTC_MIXER_PARAM_S *pstParam)
{
    if (NULL == ppMixer
        || NULL == pstParam)
    {
        return -1;
    }

    AUDIO_WEBRTC_MIXER_S *pMixer = new AUDIO_WEBRTC_MIXER_S;
    if (NULL == pMixer)
    {
        return -1;
    }

    (void)pMixer->stParam.s32Bitwidth; // only support 16bit
    static volatile int g_iId = 0;
    pMixer->stParam = *pstParam;
    pMixer->iSz10ms = SAMPLE2LENGTH((pMixer->stParam.s32Samplerate / (rtc::kNumMillisecsPerSec / 10)) * pMixer->stParam.s32Channels);
    printf("[%s:%d] 10ms size: (%d)\n", __func__, __LINE__, pMixer->iSz10ms);
    pMixer->pData = malloc(pMixer->iSz10ms);
    pMixer->pMixerImpl = webrtc::AudioConferenceMixer::Create(rtc::AtomicOps::Increment(&g_iId));

    pMixer->pCb = new AudioCallback;

    pMixer->pMixerImpl->RegisterMixedStreamCallback(pMixer->pCb);

    for (int i = 0; i < MIXER_PAR_NUM; i++)
    {
        pMixer->stParInfo[i].bEnable = false;

        pMixer->stParInfo[i].pPar = new Participant(i, pMixer->stParam.s32Samplerate, pMixer->stParam.s32Channels);
        pMixer->pMixerImpl->SetMixabilityStatus(pMixer->stParInfo[i].pPar, false);
    }

    *ppMixer = pMixer;

    return 0;
}

DSP_S32 Audio_Webrtc_DestroyMixGrp(AUDIO_WEBRTC_MIXER_S *pMixer)
{
    if (NULL == pMixer)
    {
        return -1;
    }

    for (int i = 0; i < MIXER_PAR_NUM; i++)
    {
        // destroy all chn when grp to be destroyed
        Audio_Webrtc_DestroyMixChn(pMixer, i);

        delete pMixer->stParInfo[i].pPar;
    }

    pMixer->pMixerImpl->UnRegisterMixedStreamCallback();
    delete pMixer->pMixerImpl;
    delete pMixer->pCb;
    free(pMixer->pData);
    delete pMixer;

    return 0;
}

static size_t Aproc_Buffer_Size(void *handle);
static void Aproc_Buffer_ReadFromEnd(void *handle, int16_t *output, size_t output_length);
DSP_S32 Audio_Webrtc_CreateMixChn(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_S32 *ps32Id, AUDIO_WEBRTC_MIXER_PARAM_S *pstParam)
{
    if (NULL == pMixer
        || NULL == ps32Id
        || NULL == pstParam)
    {
        return -1;
    }

    int i;
    for (i = 0; i < MIXER_PAR_NUM; i++)
    {
        if (!pMixer->stParInfo[i].bEnable)
        {
            pMixer->stParInfo[i].i64PtsErr = 0;
            pMixer->stParInfo[i].stParam = *pstParam;
#if defined(MIXER_USING_RESAMPLER)
            pMixer->stParInfo[i].u32Seq = 0;
            pMixer->stParInfo[i].u64PtsOld = 0;
            printf("[%s:%d] samplerate: src(%d), dst(%d)\n", __func__, __LINE__,
                   pMixer->stParInfo[i].stParam.s32Samplerate, pMixer->stParam.s32Samplerate);
            pMixer->stParInfo[i].iSz30ms = pMixer->iSz10ms * (3 + 3); // Aproc need 30 ms sample, and may expand
            pMixer->stParInfo[i].pData = malloc(pMixer->stParInfo[i].iSz30ms);
#endif
            pMixer->stParInfo[i].u32Size = MIXER_PAR_BUFDEPTH * pMixer->iSz10ms;
            // pMixer->stParInfo[i].pInput = WebRtc_CreateBuffer(pMixer->stParInfo[i].u32Size, pMixer->stParInfo[i].stParam.s32Channels);
            // WebRtc_InitBuffer(pMixer->stParInfo[i].pInput);
            AudioWebrtcBufferParam stBufferParam;
            stBufferParam.num_channels_ = pMixer->stParInfo[i].stParam.s32Channels;
            Audio_Webrtc_Buffer_Create(&pMixer->stParInfo[i].pInput, &stBufferParam);
#if defined(MIXER_USING_RESAMPLER)
            /* pMixer->stParInfo[i].pResampler =
                new webrtc::Resampler(pMixer->stParInfo[i].stParam.s32Samplerate,
                                      pMixer->stParam.s32Samplerate,
                                      pMixer->stParInfo[i].stParam.s32Channels);
            // pMixer->stParInfo[i].pResampler->Reset(pMixer->stParInfo[i].stParam.s32Samplerate,
            //                                        pMixer->stParam.s32Samplerate,
            //                                        pMixer->stParInfo[i].stParam.s32Channels); // not ResetIfNeeded, force reset */
            AudioWebrtcProcParam stAprocParam;
            stAprocParam.fs_hz_in_ = pMixer->stParInfo[i].stParam.s32Samplerate;
            stAprocParam.num_channels_ = pMixer->stParInfo[i].stParam.s32Channels;
            stAprocParam.enable_resampler_ = true;
            stAprocParam.fs_hz_out_ = pMixer->stParam.s32Samplerate;
            stAprocParam.enable_neteq_ = true;
            stAprocParam.buffer_handle_ = pMixer->stParInfo[i].pInput;
            stAprocParam.Buffer_Size = Aproc_Buffer_Size;
            stAprocParam.Buffer_ReadFromEnd = Aproc_Buffer_ReadFromEnd;
            Audio_Webrtc_Proc_Create(&pMixer->stParInfo[i].pAproc, &stAprocParam);
#endif

            pMixer->pMixerImpl->SetMixabilityStatus(pMixer->stParInfo[i].pPar, true);

            pMixer->stParInfo[i].bEnable = true;

            break;
        }
    }

    if (i < MIXER_PAR_NUM)
    {
        *ps32Id = i;

        return 0;
    }
    else
    {
        *ps32Id = -1;

        return -1;
    }
}

DSP_S32 Audio_Webrtc_DestroyMixChn(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_S32 s32Id)
{
    if (NULL == pMixer
        || s32Id < 0)
    {
        return -1;
    }

    if (s32Id < MIXER_PAR_NUM)
    {
        if (pMixer->stParInfo[s32Id].bEnable)
        {
            pMixer->stParInfo[s32Id].bEnable = false;

            pMixer->pMixerImpl->SetMixabilityStatus(pMixer->stParInfo[s32Id].pPar, false);
            // |Buffer| destroy here may race with |Audio_Webrtc_GetMixGrpFrame|
            // move it into |Audio_Webrtc_DestroyMixGrp|, only |Buffer| clear need to be done here
            // WebRtc_FreeBuffer(pMixer->stParInfo[s32Id].pInput);
            // // WebRtc_InitBuffer(pMixer->stParInfo[s32Id].pInput); // clear buffer, no need
            Audio_Webrtc_Buffer_Destroy(pMixer->stParInfo[s32Id].pInput);
            pMixer->stParInfo[s32Id].pInput = NULL;
#if defined(MIXER_USING_RESAMPLER)
            /* delete pMixer->stParInfo[s32Id].pResampler; */
            Audio_Webrtc_Proc_Destroy(pMixer->stParInfo[s32Id].pAproc);

            free(pMixer->stParInfo[s32Id].pData);
#endif
        }

        return 0;
    }
    else
    {
        return -1;
    }
}

DSP_S32 Audio_Webrtc_SentMixChnFrame(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_S32 s32Id, DSP_PUB_DATAINFO_S *pstData)
{
    if (NULL == pMixer
        || s32Id < 0
        || NULL == pstData)
    {
        return -1;
    }

    // push to buffer, cannot fill
    if (s32Id < MIXER_PAR_NUM)
    {
        // if (!pMixer->stParInfo[s32Id].bEnable) // no need
        // {
        //     return -1;
        // }

        size_t length;
        // length = WebRtc_available_write(pMixer->stParInfo[s32Id].pInput);
        Audio_Webrtc_Buffer_Size(pMixer->stParInfo[s32Id].pInput, &length);
        length = (length < pMixer->stParInfo[s32Id].u32Size) ? (pMixer->stParInfo[s32Id].u32Size - length) : 0;
        length = SAMPLE2LENGTH(length);
        if (length < pstData->s32Len)
        {
            printf("[%s] (%d): not enough buf (%d > %d)\n", __func__, s32Id,
                   pstData->s32Len, length);

            return -1;
        }
        else
        {
            // static FILE *pFile[MIXER_PAR_NUM] = {NULL, NULL, NULL, NULL};
            // if (NULL == pFile[s32Id])
            // {
            //     char sPath[32];
            //     snprintf(sPath, sizeof(sPath), "webrtc_mixer_dump_in_%d.pcm", s32Id);
            //     pFile[s32Id] = fopen(sPath, "wb");
            // }
            // fwrite(pstData->pVirAddr, 1, pstData->s32Len, pFile[s32Id]);

            int iReadSz, iOutSz;
            void *pReadData, *pOutData;
            iReadSz = LENGTH2SAMPLE(pstData->s32Len);
            pReadData = pstData->pVirAddr;
#if defined(MIXER_USING_RESAMPLER)
            /* if (pMixer->stParInfo[s32Id].stParam.s32Samplerate != pMixer->stParam.s32Samplerate)
            {
                // resample to the required rate
                iOutSz = LENGTH2SAMPLE(pMixer->iSz10ms);
                pOutData = pMixer->stParInfo[s32Id].pData;
                pMixer->stParInfo[s32Id].pResampler->Push((const int16_t *)pReadData, iReadSz,
                        (int16_t *)pOutData, (size_t)iOutSz,
                        (size_t &)iOutSz);
            }
            else
            {
                // resampler will copy data
                iOutSz = iReadSz;
                pOutData = pReadData;
            } */
            {
                iOutSz = LENGTH2SAMPLE(pMixer->stParInfo[s32Id].iSz30ms); // Aproc need 30 ms
                pOutData = pMixer->stParInfo[s32Id].pData;
                Audio_Webrtc_Proc_Process(pMixer->stParInfo[s32Id].pAproc,
                                          (const int16_t *)pReadData, (size_t)iReadSz,
                                          (int16_t *)pOutData, (size_t *)&iOutSz);
            }
            // printf("[%s:%d] in(%d) out(%d)\n", __func__, __LINE__, iReadSz, iOutSz);
#else
            iOutSz = iReadSz;
            pOutData = pReadData;
#endif
            // WebRtc_WriteBuffer(pMixer->stParInfo[s32Id].pInput, pOutData, SAMPLE2LENGTH(iOutSz));
            Audio_Webrtc_Buffer_PushBack(pMixer->stParInfo[s32Id].pInput,
                                         (const int16_t *)pOutData, (size_t)iOutSz);

            return 0;
        }
    }
    else
    {
        return -1;
    }

    return 0;
}

DSP_S32 Audio_Webrtc_GetMixGrpFrame(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_PUB_DATAINFO_S *pstData)
{
    if (NULL == pMixer
        || NULL == pstData)
    {
        return -1;
    }

    bool bCanProcess = true;
    // check whether time is ok
    int64_t timeLeft;
    if ((timeLeft = pMixer->pMixerImpl->TimeUntilNextProcess()) != 0)
    {
        // printf("left (%lld) ms\n", timeLeft);
        bCanProcess = false;

        // return timeLeft;
    }

    // check whether all participants are ready
    for (int i = 0; i < MIXER_PAR_NUM; i++)
    {
        Participant *pPar = pMixer->stParInfo[i].pPar;
        // RingBuffer *pInput = pMixer->stParInfo[i].pInput;
        AudioWebrtcBuffer *pInput = pMixer->stParInfo[i].pInput;
        if (pMixer->stParInfo[i].bEnable)
        {
            size_t length;
            // length = WebRtc_available_read(pInput);
            Audio_Webrtc_Buffer_Size(pInput, &length);
            length = SAMPLE2LENGTH(length);
            if (length < pMixer->iSz10ms)
            {
                if (pMixer->pMixerImpl->MixabilityStatus(*pPar))
                {
                    if (0 == pMixer->stParInfo[i].i64PtsErr) // first time
                    {
                        pMixer->stParInfo[i].i64PtsErr = rtc::TimeMillis();
                    }
                    // iErrCnt is useless, for timeLeft won't change if bCanProcess is false
                    if (rtc::TimeSince(pMixer->stParInfo[i].i64PtsErr) > (MIXER_PAR_BUFDEPTH - 2) * 10)
                    {
                        // int64_t cur = rtc::TimeMillis();
                        // printf("(%d): first(%lld), current(%lld), diff(%lld)\n", i,
                        //        pMixer->stParInfo[i].i64PtsErr, cur, cur - pMixer->stParInfo[i].i64PtsErr);
                        printf("(%d): no input, stop\n", i);
                        pMixer->pMixerImpl->SetMixabilityStatus(pPar, false);
                    }
                    else
                    {
                        bCanProcess = false;

                        break; // break here, no need to check all participants
                    }
                }
            }
            else
            {
                pMixer->stParInfo[i].i64PtsErr = 0; // clear
                if (!pMixer->pMixerImpl->MixabilityStatus(*pPar))
                {
                    printf("(%d): incoming input, restart\n", i);
                    pMixer->pMixerImpl->SetMixabilityStatus(pPar, true);
                }
            }
        }
        else
        {
            // force false, for sync problem with Audio_Webrtc_DestroyMixChn
            pMixer->pMixerImpl->SetMixabilityStatus(pPar, false);
        }
    }

    if (bCanProcess)
    {
        for (int i = 0; i < MIXER_PAR_NUM; i++)
        {
            Participant *pPar = pMixer->stParInfo[i].pPar;
            // RingBuffer *pInput = pMixer->stParInfo[i].pInput;
            AudioWebrtcBuffer *pInput = pMixer->stParInfo[i].pInput;
            if (pMixer->pMixerImpl->MixabilityStatus(*pPar)) // no need to check pMixer->stParInfo[i].bEnable
            {
                int iReadSz;
                void *pReadData;

                iReadSz = LENGTH2SAMPLE(pMixer->iSz10ms);
                pReadData = pMixer->pData;
                // WebRtc_ReadBuffer(pInput, NULL, pReadData, SAMPLE2LENGTH(iReadSz));
                Audio_Webrtc_Buffer_PopFront(pInput, (int16_t *)pReadData, iReadSz);

                // static FILE *pFile[MIXER_PAR_NUM] = {NULL, NULL, NULL, NULL};
                // if (NULL == pFile[i])
                // {
                //     char sPath[32];
                //     snprintf(sPath, sizeof(sPath), "webrtc_mixer_dump_out_%d.pcm", i);
                //     pFile[i] = fopen(sPath, "wb");
                // }
                // fwrite(pReadData, 1, SAMPLE2LENGTH(iReadSz), pFile[i]);
                // printf("[%s:%d] (%d, %p)filldata(%p, %d)\n", __func__, __LINE__, i, (void *)pPar, pReadData, iReadSz);

                pPar->filldata((const int16_t *)pReadData, (size_t)iReadSz);
            }
        }

        pMixer->pMixerImpl->Process();
        pstData->pVirAddr = pMixer->pCb->m_audioFrame.data();
        pstData->s32Len = pMixer->iSz10ms;
        pstData->u64Pts = 0; // TODO rtc::TimeNanos();
        pstData->u32Seq = 0;
        pstData->enDataType = DSP_DATATYPE_AFRAME;
        pstData->stAFrameInfo.enSoundmode =
            (2 == pMixer->stParam.s32Channels) ? DSP_PIXELFORMAT_STEREO : DSP_PIXELFORMAT_MONO;
        pstData->stAFrameInfo.enBitwidth = (DSP_BITWIDTH_E)pMixer->stParam.s32Bitwidth;

        // static FILE *pOut = NULL;
        // if (NULL == pOut)
        // {
        //     pOut = fopen("webrtc_mixer_dump_out_mixed.pcm", "wb");
        // }
        // fwrite(pstData->pVirAddr, 1, pstData->s32Len, pOut);
        // printf("[%s:%d] Process (%p, %d)\n", __func__, __LINE__, pstData->pVirAddr, pstData->s32Len);

        return 0;
    }
    else
    {
        // printf("[%s:%d] cannot Process\n", __func__, __LINE__);

        return -1;
    }
}

DSP_S32 Audio_Webrtc_ReleaseMixGrpFrame(AUDIO_WEBRTC_MIXER_S *pMixer, DSP_PUB_DATAINFO_S *pstData)
{
    if (NULL == pMixer
        || NULL == pstData)
    {
        return -1;
    }

    // do nothing

    return 0;
}

static size_t Aproc_Buffer_Size(void *handle)
{
    AudioWebrtcBuffer *buffer_handle = (AudioWebrtcBuffer *)handle;
    size_t length = 0;
    Audio_Webrtc_Buffer_Size(buffer_handle, &length);

    return length;
}
static void Aproc_Buffer_ReadFromEnd(void *handle, int16_t *output, size_t output_length)
{
    AudioWebrtcBuffer *buffer_handle = (AudioWebrtcBuffer *)handle;
    Audio_Webrtc_Buffer_PopBack(buffer_handle, output, output_length);
}
