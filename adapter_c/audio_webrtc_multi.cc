
#include "audio_webrtc_multi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "audio_webrtc_proc.h"

#define MULTI_USING_RESAMPLER
#define MULTI_CHN_NUM       (4)
#define MULTI_CHN_CACHE_NUM (8)
typedef struct AUDIO_WEBRTC_CHN_BUF_INFO
{
    DSP_S32             s32WritePosition;  //上一次写入位置 初始值1
    // DSP_S32             s32ReadPosition;   //下一个读取位置 初始值1
    // DSP_U32             u32ReadableNum;    //当前可读取帧数 初始值0
    DSP_U32             u32WriteCnt;       //已写入计数 初始值0
    // DSP_U32             u32ReadCnt;        //已读取计数 初始值0
    // DSP_U32             u32FullCnt;        //满计数 初始值0

    DSP_BOOL            bWrited[MULTI_CHN_CACHE_NUM];       //通道缓存帧是否已写入 1-已写入 0-未写入 初始值0
    DSP_BOOL            bOccupied[MULTI_CHN_CACHE_NUM];     //通道缓存帧是否占用 1-已占用 0-未占用 初始值0
    DSP_U32             u32FrameSeq[MULTI_CHN_CACHE_NUM];   //帧序号 初始值0
    DSP_PUB_DATAINFO_S  stData[MULTI_CHN_CACHE_NUM];        // out data after |Amul| dup
} AUDIO_WEBRTC_CHN_BUF_INFO_S;

//每个组有这样一个结构体
struct AUDIO_WEBRTC_MULTI
{
    AUDIO_WEBRTC_MULTI_PARAM_S      stParam;        // grp param
    struct
    {
        DSP_BOOL                    bChnUsed;
        DSP_U32                     u32ChnBufNum;   //创建grp时初始化 取值范围(0, MULTI_CHN_CACHE_NUM] depth
        AUDIO_WEBRTC_CHN_BUF_INFO_S stChnBufInfo;
        AUDIO_WEBRTC_MULTI_PARAM_S  stParam;        // chn param
#if defined(MULTI_USING_RESAMPLER)
        DSP_PUB_DATAINFO_S          stData;         // out data after |Aproc|
        AudioWebrtcProc             *pAproc;        // resampler
#endif
    } stChnInfo[MULTI_CHN_NUM];
};

static DSP_S32 Audio_Webrtc_BufInfoInit(AUDIO_WEBRTC_CHN_BUF_INFO_S *pstChnBufInfo);
static DSP_S32 Audio_Webrtc_AFrameAlloc(DSP_PUB_DATAINFO_S *pstData);
static DSP_S32 Audio_Webrtc_AFrameFree(DSP_PUB_DATAINFO_S *pstData);
static DSP_S32 Audio_Webrtc_AFrameDup(DSP_PUB_DATAINFO_S *pstSrc, DSP_PUB_DATAINFO_S *pstDest);
static DSP_S32 Audio_Webrtc_AFrameCmp(DSP_PUB_DATAINFO_S *pstSrc, DSP_PUB_DATAINFO_S *pstDest);

DSP_S32 Audio_Webrtc_CreateMulGrp(AUDIO_WEBRTC_MULTI_S **ppMulti, AUDIO_WEBRTC_MULTI_PARAM_S *pstParam)
{
    if (NULL == ppMulti
        || NULL == pstParam)
    {
        return -1;
    }
    DSP_U32 i;
    AUDIO_WEBRTC_MULTI_S *pMulti = new AUDIO_WEBRTC_MULTI_S;

    pMulti->stParam = *pstParam;
    (void)pMulti->stParam.s32Bitwidth; // only support 16bit

    for (i = 0; i < MULTI_CHN_NUM; i++)
    {
        pMulti->stChnInfo[i].bChnUsed = DSP_FALSE;

        pMulti->stChnInfo[i].u32ChnBufNum = 5;
        // Audio_Webrtc_BufInfoInit(&pMulti->stChnInfo[i].stChnBufInfo);
    }

    *ppMulti = pMulti;

    return 0;
}

DSP_S32 Audio_Webrtc_DestroyMulGrp(AUDIO_WEBRTC_MULTI_S *pMulti)
{
    if (NULL == pMulti)
    {
        return -1;
    }
    DSP_U32 i;

    for (i = 0; i < MULTI_CHN_NUM; i++)
    {
        Audio_Webrtc_DestroyMulChn(pMulti, i);
    }

    delete pMulti;

    return 0;
}

DSP_S32 Audio_Webrtc_CreateMulChn(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_S32 *ps32Id, AUDIO_WEBRTC_MULTI_PARAM_S *pstParam)
{
    if (NULL == pMulti
        || NULL == ps32Id
        || NULL == pstParam)
    {
        return -1;
    }
    DSP_U32 i;

    for (i = 0; i < MULTI_CHN_NUM; i++)
    {
        if (!pMulti->stChnInfo[i].bChnUsed)
        {
            Audio_Webrtc_BufInfoInit(&pMulti->stChnInfo[i].stChnBufInfo);

            pMulti->stChnInfo[i].stParam = *pstParam;
#if defined(MULTI_USING_RESAMPLER)
            pMulti->stChnInfo[i].stData.s32Len = SAMPLE2LENGTH(1 * 48 * 30); // Mono, 48 kHz, 30 ms
            printf("[%s:%d] (%d) samplerate: src(%d), dst(%d)\n", __func__, __LINE__, i,
                   pMulti->stParam.s32Samplerate, pMulti->stChnInfo[i].stParam.s32Samplerate);
            Audio_Webrtc_AFrameAlloc(&pMulti->stChnInfo[i].stData);
            AudioWebrtcProcParam stAprocParam;
            stAprocParam.fs_hz_in_ = pMulti->stParam.s32Samplerate;
            stAprocParam.num_channels_ = pMulti->stParam.s32Channels;
            stAprocParam.enable_limiter_ = false;
            stAprocParam.enable_resampler_ = true;
            stAprocParam.fs_hz_out_ = pMulti->stChnInfo[i].stParam.s32Samplerate;
            stAprocParam.enable_neteq_ = false;
            stAprocParam.buffer_handle_ = nullptr;
            stAprocParam.Buffer_Size = nullptr;
            stAprocParam.Buffer_ReadFromEnd = nullptr;
            Audio_Webrtc_Proc_Create(&pMulti->stChnInfo[i].pAproc, &stAprocParam);
#endif

            pMulti->stChnInfo[i].bChnUsed = DSP_TRUE;

            break;
        }
    }
    if (i < MULTI_CHN_NUM)
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

DSP_S32 Audio_Webrtc_DestroyMulChn(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_S32 s32Id)
{
    if (NULL == pMulti
        || s32Id < 0 || s32Id >= MULTI_CHN_NUM)
    {
        return -1;
    }
    DSP_U32 i;

    if (pMulti->stChnInfo[s32Id].bChnUsed)
    {
        pMulti->stChnInfo[s32Id].bChnUsed = DSP_FALSE;
#if defined(MULTI_USING_RESAMPLER)
        Audio_Webrtc_Proc_Destroy(pMulti->stChnInfo[s32Id].pAproc);
        Audio_Webrtc_AFrameFree(&pMulti->stChnInfo[s32Id].stData);
#endif
        for (i = 0; i < pMulti->stChnInfo[s32Id].u32ChnBufNum; i++)
        {
            if (pMulti->stChnInfo[s32Id].stChnBufInfo.bWrited[i])
            {
                if ((Audio_Webrtc_AFrameFree(&pMulti->stChnInfo[s32Id].stChnBufInfo.stData[i])) != 0)
                {
                    return -1;
                }
            }
        }
        // Audio_Webrtc_BufInfoInit(&pMulti->stChnInfo[i].stChnBufInfo);

        return 0;
    }
    else
    {
        return 0; // success if already destroyed
    }
}

DSP_S32 Audio_Webrtc_SentMulGrpFrame(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_PUB_DATAINFO_S *pstData)
{
    if (NULL == pMulti
        || NULL == pstData)
    {
        return -1;
    }
    DSP_U32 i, j;
    AUDIO_WEBRTC_CHN_BUF_INFO_S *pstChnBufInfo;

    for (i = 0; i < MULTI_CHN_NUM; i++)
    {
        if (pMulti->stChnInfo[i].bChnUsed) // 判断通道是否启用，启用则拷贝
        {
            //寻找可写入的位置
            DSP_U32 u32Next_WPos;
            DSP_S32 bFind = 0;
            pstChnBufInfo = &pMulti->stChnInfo[i].stChnBufInfo;
            u32Next_WPos = pstChnBufInfo->s32WritePosition + 1;
            for (j = 0; j < pMulti->stChnInfo[i].u32ChnBufNum; j++)
            {
                if (u32Next_WPos >= pMulti->stChnInfo[i].u32ChnBufNum)
                    u32Next_WPos = 0;
                if (!pstChnBufInfo->bOccupied[u32Next_WPos])
                {
                    if (!pstChnBufInfo->bWrited[u32Next_WPos])
                    {
                        bFind = 1; // 找到
                    }
                    else
                    {
                        bFind = 2; // 循环覆盖（一直未取，一直在发，新的覆盖旧的）
                    }
                    break;
                }
                u32Next_WPos++;
            }
            //音频数据拷贝
            // printf("[%s] (%d): u32Next_WPos is (%d)\n", __func__, i, u32Next_WPos);
            if (0 == bFind /* && j == pMulti->stChnInfo[i].u32ChnBufNum */) // 两个条件出现情况一样，只需要一个即可
            {
                // 通道满
                // pstChnBufInfo->u32FullCnt++;
                printf("[%s] (%d): buf is full, please release first!\n", __func__, i);
                return -1;
            }
            else if (1 == bFind)
            {
                ; // pstChnBufInfo->u32ReadableNum++;
            }
            else if (2 == bFind)
            {
                if ((Audio_Webrtc_AFrameFree(&pstChnBufInfo->stData[u32Next_WPos])) != 0)
                {
                    return -1;
                }
            }
            else
            {;}
            if ((Audio_Webrtc_AFrameDup(pstData, &pstChnBufInfo->stData[u32Next_WPos])) != 0)
            {
                return -1;
            }
            //更新写位置
            pstChnBufInfo->u32WriteCnt += 1;
            pstChnBufInfo->bWrited[u32Next_WPos] = DSP_TRUE;
            pstChnBufInfo->s32WritePosition = (DSP_S32)u32Next_WPos;
            pstChnBufInfo->u32FrameSeq[u32Next_WPos] = pstChnBufInfo->u32WriteCnt;
        }
    }

    return 0;
}

DSP_S32 Audio_Webrtc_GetMulChnFrame(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_S32 s32Id, DSP_PUB_DATAINFO_S *pstData)
{
    if (NULL == pMulti
        || s32Id < 0 || s32Id >= MULTI_CHN_NUM
        || NULL == pstData)
    {
        return -1;
    }
    DSP_U32 i;
    AUDIO_WEBRTC_CHN_BUF_INFO_S *pstChnBufInfo;

    if (pMulti->stChnInfo[s32Id].bChnUsed)
    {
        pstChnBufInfo = &pMulti->stChnInfo[s32Id].stChnBufInfo;
        DSP_S32 bFind = 0;
        DSP_U32 u32MinFrameSeq;
        DSP_U32 u32ReadPos;
        //寻找哪个buf可以被读出
        for (i = 0; i < pMulti->stChnInfo[s32Id].u32ChnBufNum; i++)
        {
            if (pstChnBufInfo->bWrited[i])
            {
                if (!pstChnBufInfo->bOccupied[i])
                {
                    if (0 == bFind)
                    {
                        bFind = 1; // 找到
                        u32ReadPos = i;
                        u32MinFrameSeq = pstChnBufInfo->u32FrameSeq[i];
                    }
                    else if (1 == bFind) // 找最旧的
                    {
                        if (u32MinFrameSeq < pstChnBufInfo->u32FrameSeq[i])
                        {
                            u32ReadPos = i;
                            u32MinFrameSeq = pstChnBufInfo->u32FrameSeq[i];
                        }
                    }
                }
            }
        }
        if (0 == bFind)
        {
            // 没有可读的buf
            // printf("[%s] (%d): buf is empty!\n", __func__, s32Id);
            return -1;//空
        }
        else if (1 == bFind)
        {
            // 找到
#if defined(MULTI_USING_RESAMPLER)
            void *pInData = pstChnBufInfo->stData[u32ReadPos].pVirAddr;
            size_t iInSz = LENGTH2SAMPLE(pstChnBufInfo->stData[u32ReadPos].s32Len);
            void *pOutData = pMulti->stChnInfo[s32Id].stData.pVirAddr;
            size_t iOutSz = LENGTH2SAMPLE(pMulti->stChnInfo[s32Id].stData.s32Len);
            Audio_Webrtc_Proc_Process(pMulti->stChnInfo[s32Id].pAproc,
                                      (const int16_t *)pInData, (size_t)iInSz,
                                      (int16_t *)pOutData, (size_t *)&iOutSz);
            pstData->pVirAddr = pOutData;
            pstData->s32Len = SAMPLE2LENGTH(iOutSz);

            // printf("[%s] (%d): in%p,%u; out%p,%u;\n", __func__, s32Id,
            //        pInData, SAMPLE2LENGTH(iInSz), pOutData, SAMPLE2LENGTH(iOutSz));

            // 直接在这里释放，因为resample拷贝了一次
            if ((Audio_Webrtc_AFrameFree(&pstChnBufInfo->stData[u32ReadPos])) != 0)
            {
                return -1;
            }
            pstChnBufInfo->bOccupied[u32ReadPos] = DSP_FALSE;
            pstChnBufInfo->bWrited[u32ReadPos] = DSP_FALSE;
#else
            memmove(pstData, &pstChnBufInfo->stData[u32ReadPos], sizeof(DSP_PUB_DATAINFO_S));
            pstChnBufInfo->stData[u32ReadPos].u32Seq = u32MinFrameSeq;
            pstData->u32Seq = u32MinFrameSeq; // pstChnBufInfo->u32FrameSeq[u32ReadPos];
            pstChnBufInfo->bOccupied[u32ReadPos] = DSP_TRUE;
#endif
            // pstChnBufInfo->u32ReadCnt++;
            // pstChnBufInfo->s32ReadPosition = u32ReadPos;
            // pstChnBufInfo->u32ReadableNum--;
        }
        else
        {;}

        return 0;
    }
    else
    {
        return -1;
    }
}

DSP_S32 Audio_Webrtc_ReleaseMulChnFrame(AUDIO_WEBRTC_MULTI_S *pMulti, DSP_S32 s32Id, DSP_PUB_DATAINFO_S *pstData)
{
    if (NULL == pMulti
        || s32Id < 0 || s32Id >= MULTI_CHN_NUM
        || NULL == pstData)
    {
        return -1;
    }
#if defined(MULTI_USING_RESAMPLER)
    // 在 |Audio_Webrtc_GetMulChnFrame| 内释放
    // (void)Audio_Webrtc_AFrameCmp;
#else
    DSP_U32 i;
    AUDIO_WEBRTC_CHN_BUF_INFO_S *pstChnBufInfo;

    if (pMulti->stChnInfo[s32Id].bChnUsed)
    {
        pstChnBufInfo = &pMulti->stChnInfo[s32Id].stChnBufInfo;
        for (i = 0; i < pMulti->stChnInfo[s32Id].u32ChnBufNum; i++)
        {
            if (pstChnBufInfo->bOccupied[i])
            {
                if (0 == Audio_Webrtc_AFrameCmp(&pstChnBufInfo->stData[i], pstData))
                {
                    if ((Audio_Webrtc_AFrameFree(&pstChnBufInfo->stData[i])) != 0)
                    {
                        ; // return -1;
                    }
                    pstChnBufInfo->bOccupied[i] = DSP_FALSE;
                    pstChnBufInfo->bWrited[i] = DSP_FALSE;

                    break;
                }
            }
        }

        return 0;
    }
    else
#endif
    {
        return 0;
    }
}

static DSP_S32 Audio_Webrtc_BufInfoInit(AUDIO_WEBRTC_CHN_BUF_INFO_S *pstChnBufInfo)
{
    DSP_U32 i;

    pstChnBufInfo->s32WritePosition    = -1;
    // pstChnBufInfo->s32ReadPosition     = -1;
    // pstChnBufInfo->u32ReadableNum      = 0;
    pstChnBufInfo->u32WriteCnt         = 0;
    // pstChnBufInfo->u32ReadCnt          = 0;
    // pstChnBufInfo->u32FullCnt          = 0;
    for (i = 0; i < MULTI_CHN_CACHE_NUM; i++)
    {
        pstChnBufInfo->bWrited[i]      = DSP_FALSE;
        pstChnBufInfo->bOccupied[i]    = DSP_FALSE;
        pstChnBufInfo->u32FrameSeq[i]  = 0;
        memset(&pstChnBufInfo->stData[i], 0, sizeof(pstChnBufInfo->stData[i]));
    }

    return 0;
}

static DSP_S32 Audio_Webrtc_AFrameAlloc(DSP_PUB_DATAINFO_S *pstData)
{
    pstData->enDataType = DSP_DATATYPE_AFRAME;
    pstData->pVirAddr = malloc(pstData->s32Len);

    return 0;
}
static DSP_S32 Audio_Webrtc_AFrameFree(DSP_PUB_DATAINFO_S *pstData)
{
    free(pstData->pVirAddr);
    pstData->pVirAddr = NULL;

    return 0;
}
static DSP_S32 Audio_Webrtc_AFrameDup(DSP_PUB_DATAINFO_S *pstSrc, DSP_PUB_DATAINFO_S *pstDest)
{
    memmove(pstDest, pstSrc, sizeof(DSP_PUB_DATAINFO_S));
    Audio_Webrtc_AFrameAlloc(pstDest);
    memmove(pstDest->pVirAddr, pstSrc->pVirAddr, pstDest->s32Len);

    return 0;
}
static DSP_S32 Audio_Webrtc_AFrameCmp(DSP_PUB_DATAINFO_S *pstSrc, DSP_PUB_DATAINFO_S *pstDest)
{
    // printf("[%s] (%u): src%p,%d; (%u): dest%p,%d;\n", __func__,
    //        pstSrc->u32Seq, pstSrc->pVirAddr, pstSrc->s32Len, pstDest->u32Seq, pstDest->pVirAddr, pstDest->s32Len);
    return (/* (pstDest->pVirAddr == pstSrc->pVirAddr) && */ (pstDest->s32Len == pstSrc->s32Len) \
            && (pstDest->u32Seq == pstSrc->u32Seq)) ? 0 : -1;
}
