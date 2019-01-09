
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "adapter_c/audio_webrtc_proc.h"

#define USING_SYNCBUFFER

#if defined(USING_SYNCBUFFER)
#include "adapter_c/audio_webrtc_buffer.h"
static size_t SyncBuffer_Size(void *handle)
{
    AudioWebrtcBuffer *syncbuffer_handle = (AudioWebrtcBuffer *)handle;
    size_t length = 0;
    Audio_Webrtc_Buffer_Size(syncbuffer_handle, &length);

    return length;
}
static void SyncBuffer_ReadFromEnd(void *handle, int16_t *output, size_t output_length)
{
    AudioWebrtcBuffer *syncbuffer_handle = (AudioWebrtcBuffer *)handle;
    Audio_Webrtc_Buffer_PopBack(syncbuffer_handle, output, output_length);
}
#else
static size_t demo_buffer_size_ = 0;
static size_t Demo_Buffer_Size(void *handle)
{
    return (demo_buffer_size_ += 320); // same as kInputSamples
    (void)handle;
}
static void Demo_Buffer_ReadFromEnd(void *handle, int16_t *output, size_t output_length)
{
    memset(output, 0x00, output_length * sizeof(int16_t));
    demo_buffer_size_ -= output_length;
    (void)handle;
}
#endif

int main(void)
{
#if defined(USING_SYNCBUFFER)
    AudioWebrtcBuffer *syncbuffer_handle;
    AudioWebrtcBufferParam syncbuffer_parameter =
    {
        .num_channels_ = 1, // same as AudioWebrtcProcParam
    };
    Audio_Webrtc_Buffer_Create(&syncbuffer_handle, &syncbuffer_parameter);
#endif
    AudioWebrtcProc *handle;
    AudioWebrtcProcParam parameter =
    {
        .fs_hz_in_ = 16000,
        .num_channels_ = 1,

        .enable_limiter_ = false,

        .enable_resampler_ = true,
        .fs_hz_out_ = 32000,

        .enable_neteq_ = false,
#if defined(USING_SYNCBUFFER)
        .buffer_handle_ = syncbuffer_handle,
        .Buffer_Size = SyncBuffer_Size,
        .Buffer_ReadFromEnd = SyncBuffer_ReadFromEnd,
#else
        .buffer_handle_ = NULL,
        .Buffer_Size = Demo_Buffer_Size,
        .Buffer_ReadFromEnd = Demo_Buffer_ReadFromEnd,
#endif
    };
    Audio_Webrtc_Proc_Create(&handle, &parameter);

    FILE *pfile1 = fopen("demos/samples/1_16000_16_1.pcm", "rb");
    if (NULL == pfile1)
        perror("fopen pfile1:");
    FILE *pfile2 = fopen("aproc_32000.pcm", "wb");
    if (NULL == pfile2)
        perror("fopen pfile2:");
    const size_t kInputSamples = 320, kOutputSamples = (kInputSamples << 3), kFuzzerSamples = 10;
    size_t input_samples = kInputSamples + kFuzzerSamples,
           output_samples = kOutputSamples,
           fuzzer = 0;
    int16_t *input_buffer = (int16_t *)malloc(input_samples * sizeof(int16_t)),
             *output_buffer = (int16_t *)malloc(output_samples * sizeof(int16_t));

    while (pfile1 != NULL && !feof(pfile1))
    {
        // with syncbuffer, fuzzer can be handled
        // without syncbuffer, fuzzer cannot be handled (sound isn't continuous)
        input_samples = kInputSamples + ((++fuzzer) % kFuzzerSamples /* - (kFuzzerSamples >> 1) */);
        if (pfile1 != NULL)
            fread(input_buffer, 1, input_samples * sizeof(int16_t), pfile1);

        size_t output_samples = kOutputSamples;
        Audio_Webrtc_Proc_Process(handle,
                                  input_buffer, input_samples,
                                  output_buffer, &output_samples);
#if defined(USING_SYNCBUFFER)
        Audio_Webrtc_Buffer_PushBack(syncbuffer_handle, output_buffer, output_samples);
        if (pfile2 != NULL)
        {
            size_t length = (kInputSamples << 1) - 80;
            Audio_Webrtc_Buffer_PopFront(syncbuffer_handle, output_buffer, length);
            fwrite(output_buffer, 1, length * sizeof(int16_t), pfile2);
        }
#else
        if (pfile2 != NULL)
            fwrite(output_buffer, 1, output_samples * sizeof(int16_t), pfile2);
#endif
    }
#if defined(USING_SYNCBUFFER)
    if (pfile2 != NULL)
    {
        size_t length = 0;
        Audio_Webrtc_Buffer_Size(syncbuffer_handle, &length);
        printf("sync buffer remain samples: (%d)\n", length);
        while (length > 0)
        {
            size_t length_tmp = (length > kOutputSamples) ? kOutputSamples : length;
            Audio_Webrtc_Buffer_PopFront(syncbuffer_handle, output_buffer, length_tmp);
            fwrite(output_buffer, 1, length_tmp * sizeof(int16_t), pfile2);
            length -= length_tmp;
        }
        // fflush(pfile2);
        Audio_Webrtc_Buffer_Destroy(syncbuffer_handle);
    }
#endif
    if (pfile2 != NULL)
        fclose(pfile2);
    if (pfile1 != NULL)
        fclose(pfile1);
    free(output_buffer), free(input_buffer);
    Audio_Webrtc_Proc_Destroy(handle);

    return 0;
}

