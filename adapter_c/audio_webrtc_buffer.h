
#ifndef _AUDIO_WEBRTC_BUFFER_H
#define _AUDIO_WEBRTC_BUFFER_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct AudioWebrtcBuffer AudioWebrtcBuffer;
typedef struct AudioWebrtcBufferParam AudioWebrtcBufferParam;

int Audio_Webrtc_Buffer_Create(AudioWebrtcBuffer **handle,
                               AudioWebrtcBufferParam *parameter);
int Audio_Webrtc_Buffer_Destroy(AudioWebrtcBuffer *handle);
int Audio_Webrtc_Buffer_Size(AudioWebrtcBuffer *handle,
                             size_t *length);
int Audio_Webrtc_Buffer_PopBack(AudioWebrtcBuffer *handle,
                                int16_t *buffer, size_t length); // ReadFromEnd
int Audio_Webrtc_Buffer_PopFront(AudioWebrtcBuffer *handle,
                                 int16_t *buffer, size_t length); // Read(FromBegin)
int Audio_Webrtc_Buffer_PushBack(AudioWebrtcBuffer *handle,
                                 const int16_t *buffer, size_t length); // Write(ToEnd)
int Audio_Webrtc_Buffer_PushFront(AudioWebrtcBuffer *handle,
                                  const int16_t *buffer, size_t length); // WriteToBegin

struct AudioWebrtcBufferParam
{
    int num_channels_;
};


// packet buffer
typedef struct AudioWebrtcPacketBuffer AudioWebrtcPacketBuffer;
typedef struct AudioWebrtcPacketBufferParam AudioWebrtcPacketBufferParam;

int Audio_Webrtc_PacketBuffer_Create(AudioWebrtcPacketBuffer **handle,
                                     AudioWebrtcPacketBufferParam *parameter);
int Audio_Webrtc_PacketBuffer_Destroy(AudioWebrtcPacketBuffer *handle);
// length is remain_len, both in/out; buffer is offset of origin
// -1: error, 0: success, 1: length not enough
int Audio_Webrtc_PacketBuffer_Process(AudioWebrtcPacketBuffer *handle,
                                      const void *buffer, size_t *length);
// whether if user want to use this buffer again or skip this buffer
int Audio_Webrtc_PacketBuffer_MarkUsed(AudioWebrtcPacketBuffer *handle,
                                       bool used);

struct AudioWebrtcPacketBufferParam
{
    void *buffer_; // reduce copy
    // buffer |length| is |data_len_|+|head_len_|
    size_t data_len_;
    void *head_data_;
    size_t head_len_;
};

#if defined(__cplusplus)
}
#endif

#endif /* _AUDIO_WEBRTC_BUFFER_H */
