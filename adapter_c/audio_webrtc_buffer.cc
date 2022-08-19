
#include "audio_webrtc_buffer.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "modules/audio_coding/neteq/audio_multi_vector.h"

// NOTE(lgY): |sync_buffer_|
/* // |Buffer_Size| is used as parameter in |buffer_level_filter_|->Update
// |Buffer_ReadFromEnd| is used to borrow samples to acquire |required_samples_|
struct AudioWebrtcBuffer
{
    void *buffer_handle_;
    // callbacks
    size_t (*Buffer_Size)(void *); // used size, Size()
    void (*Buffer_ReadFromEnd)(void *, void *, size_t); // borrow, PopBack()
    void (*Buffer_ReadFromBegin)(void *, void *, size_t); // read, PopFront()
    void (*Buffer_WriteToEnd)(void *, size_t, size_t); // write, PushBack()
    void (*Buffer_WriteToBegin)(void *, size_t, size_t); // insert, PushFront()
}; */

struct AudioWebrtcBuffer
{
    webrtc::AudioMultiVector *buffer_;
};

int Audio_Webrtc_Buffer_Create(AudioWebrtcBuffer **handle,
                               AudioWebrtcBufferParam *parameter)
{
    if (NULL == handle
        || NULL == parameter)
    {
        return -1;
    }

    int return_code;
    AudioWebrtcBuffer *handle_tmp = NULL;
    do
    {
        handle_tmp =
            (AudioWebrtcBuffer *)malloc(sizeof(AudioWebrtcBuffer));
        if (NULL == handle_tmp)
        {
            return_code = -1;
            break;
        }

        handle_tmp->buffer_ =
            new webrtc::AudioMultiVector(parameter->num_channels_);
        if (NULL == handle_tmp->buffer_)
        {
            free(handle_tmp);
            handle_tmp = NULL;
            return_code = -1;
            break;
        }

        return_code = 0;
    }
    while (0);

    *handle = handle_tmp;

    return return_code;
}

int Audio_Webrtc_Buffer_Destroy(AudioWebrtcBuffer *handle)
{
    if (NULL == handle)
    {
        return -1;
    }

    // TODO check |buffer_|

    int return_code;
    do
    {
        delete handle->buffer_;
        free(handle);

        return_code = 0;
    }
    while (0);

    return return_code;
}

int Audio_Webrtc_Buffer_Size(AudioWebrtcBuffer *handle,
                             size_t *length)
{
    if (NULL == handle
        || NULL == length)
    {
        return -1;
    }

    // TODO check |buffer_|

    int return_code;
    do
    {
        *length = handle->buffer_->Size();

        return_code = 0;
    }
    while (0);

    return return_code;
}

int Audio_Webrtc_Buffer_PopBack(AudioWebrtcBuffer *handle,
                                int16_t *buffer, size_t length)
{
    if (NULL == handle
        || NULL == buffer || 0 == length)
    {
        return -1;
    }

    // TODO check |buffer_|

    int return_code;
    do
    {
        handle->buffer_->ReadInterleavedFromEnd(length, buffer);
        handle->buffer_->PopBack(length);

        return_code = 0;
    }
    while (0);

    return return_code;
}

int Audio_Webrtc_Buffer_PopFront(AudioWebrtcBuffer *handle,
                                 int16_t *buffer, size_t length)
{
    if (NULL == handle
        || NULL == buffer || 0 == length)
    {
        return -1;
    }

    // TODO check |buffer_|

    int return_code;
    do
    {
        handle->buffer_->ReadInterleaved(length, buffer);
        handle->buffer_->PopFront(length);

        return_code = 0;
    }
    while (0);

    return return_code;
}

int Audio_Webrtc_Buffer_PushBack(AudioWebrtcBuffer *handle,
                                 const int16_t *buffer, size_t length)
{
    if (NULL == handle
        || NULL == buffer || 0 == length)
    {
        return -1;
    }

    // TODO check |buffer_|

    int return_code;
    do
    {
        handle->buffer_->PushBackInterleaved(buffer, length);

        return_code = 0;
    }
    while (0);

    return return_code;
}

int Audio_Webrtc_Buffer_PushFront(AudioWebrtcBuffer *handle,
                                  const int16_t *buffer, size_t length)
{
    if (NULL == handle
        || NULL == buffer || 0 == length)
    {
        return -1;
    }

    // TODO check |buffer_|

    int return_code;
    do
    {
        // TODO(lgY): not support
        // handle->buffer_->PushFront(buffer, length);

        return_code = 0;
    }
    while (0);

    return return_code;
}


// NOTE(lgY): packet buffer

struct AudioWebrtcPacketBuffer
{
    void *buffer_;
    size_t length_;
    size_t head_len_;
    size_t used_len_;
};

int Audio_Webrtc_PacketBuffer_Create(AudioWebrtcPacketBuffer **handle,
                                     AudioWebrtcPacketBufferParam *parameter)
{
    if (NULL == handle
        || NULL == parameter)
    {
        return -1;
    }

    int return_code;
    AudioWebrtcPacketBuffer *handle_tmp = NULL;
    do
    {
        handle_tmp =
            (AudioWebrtcPacketBuffer *)malloc(sizeof(AudioWebrtcPacketBuffer));
        if (NULL == handle_tmp)
        {
            return_code = -1;
            break;
        }

        handle_tmp->buffer_ = parameter->buffer_;
        handle_tmp->head_len_ = parameter->head_len_;
        if (NULL == handle_tmp->buffer_
            || 0 == parameter->data_len_
            || (handle_tmp->head_len_ != 0 && NULL == parameter->head_data_))
        {
            free(handle_tmp);
            handle_tmp = NULL;
            return_code = -1;
            break;
        }
        memmove(handle_tmp->buffer_, parameter->head_data_, handle_tmp->head_len_);
        handle_tmp->length_ = handle_tmp->head_len_ + parameter->data_len_;
        handle_tmp->used_len_ = 0;

        return_code = 0;
    }
    while (0);

    *handle = handle_tmp;

    return return_code;
}

int Audio_Webrtc_PacketBuffer_Destroy(AudioWebrtcPacketBuffer *handle)
{
    if (NULL == handle)
    {
        return -1;
    }

    int return_code;
    do
    {
        free(handle);

        return_code = 0;
    }
    while (0);

    return return_code;
}

int Audio_Webrtc_PacketBuffer_Process(AudioWebrtcPacketBuffer *handle,
                                      const void *buffer, size_t *length)
{
    if (NULL == handle
        || NULL == buffer || NULL == length)
    {
        return -1;
    }

    int return_code;
    do
    {
        size_t dst_used_len = handle->used_len_ + handle->head_len_;
        size_t dst_remain_len = handle->length_ - dst_used_len;
        if (0 == dst_remain_len)
        {
            handle->used_len_ = 0;
            return_code = 0; // decode this frame before call this func
            break; // re-calc next time
        }

        size_t src_used_len = 0;
        size_t src_remain_len = *length;
        if (0 == src_remain_len)
        {
            return_code = 1; // call this func again after input another packet
            break;
        }

        size_t move_len = dst_remain_len > src_remain_len ? src_remain_len : dst_remain_len; // std::min
        memmove((void *)((int8_t *)handle->buffer_ + dst_used_len), // src
                (void *)((int8_t *)buffer + src_used_len), // dst
                move_len);
        handle->used_len_ += move_len; // src
        *length -= move_len; // dst
    }
    while (1); // re-use dst_remain_len check

    return return_code;
}

int Audio_Webrtc_PacketBuffer_MarkUsed(AudioWebrtcPacketBuffer *handle,
                                       bool used)
{
    if (NULL == handle)
    {
        return -1;
    }

    int return_code;
    do
    {
        // if current buffer is used, |used_len_| is 0, otherwise is |data_len_|
        handle->used_len_ = used ? 0 : handle->length_ - handle->head_len_;

        return_code = 0;
    }
    while (0);

    return return_code;
}
