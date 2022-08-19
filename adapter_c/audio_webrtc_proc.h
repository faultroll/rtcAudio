
#ifndef _AUDIO_WEBRTC_PROC_H
#define _AUDIO_WEBRTC_PROC_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct AudioWebrtcProc AudioWebrtcProc;
typedef struct AudioWebrtcProcParam AudioWebrtcProcParam;

int Audio_Webrtc_Proc_Create(AudioWebrtcProc **handle,
                             AudioWebrtcProcParam *parameter);
int Audio_Webrtc_Proc_Destroy(AudioWebrtcProc *handle);
int Audio_Webrtc_Proc_Process(AudioWebrtcProc *handle,
                              const int16_t *input, size_t input_length,
                              int16_t *output, size_t *output_length);

struct AudioWebrtcProcParam
{
    int fs_hz_in_;
    int num_channels_;
    struct
    {
        // agc
        bool enable_limiter_;
    };
    struct
    {
        bool enable_resampler_;
        int fs_hz_out_;
    };
    /* struct
    {
        // ns/transient
        bool enable_suppressor_;
    }; */
    struct
    {
        // only support 8/16/32/48 khz when |enable_neteq_|
        // |output_length| must be at least twice of 30ms sample length (60ms) when |enable_neteq_|
        bool enable_neteq_;
        void *buffer_handle_; // |sync_buffer_|
        size_t (*Buffer_Size)(void *); // used size
        void (*Buffer_ReadFromEnd)(void *, int16_t *, size_t); // borrow
    };
};

#define LENGTH2SAMPLE(_len) ((size_t)(_len) >> 1) // ((size_t)(_len) / sizeof(int16_t))
#define SAMPLE2LENGTH(_sample) ((size_t)(_sample) << 1) // ((size_t)(_sample) * sizeof(int16_t))

#if defined(__cplusplus)
}
#endif

#endif /* _AUDIO_WEBRTC_PROC_H */
