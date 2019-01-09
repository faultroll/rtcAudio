
#include "audio_webrtc_proc.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// TODO(s)
// DONE 1. name (webrtc-like)
// DONE 2. remove background_noise
// DONE 3. replace delay_manager with m75

// using namespace webrtc;

// header
#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct AprocLimiter AprocLimiter;
typedef struct AprocLimiterParam AprocLimiterParam;

int Aproc_Limiter_Create(AprocLimiter **handle,
                         AprocLimiterParam *parameter
                        );
int Aproc_Limiter_Destroy(AprocLimiter *handle
                         );
int Aproc_Limiter_Process(AprocLimiter *handle,
                          const int16_t *input, size_t input_length,
                          int16_t *output, size_t *output_length
                         );

struct AprocLimiterParam
{
    int fs_hz_;
    int num_channels_;
};

#if defined(__cplusplus)
};
#endif

// source
#include "modules/audio_processing/agc/gain_control_impl.h"
#include "common_audio/include/audio_util.h"
#include "modules/audio_processing/audio_buffer.h"

struct AprocLimiter
{
    // TODO remove crit
    rtc::CriticalSection *crit_render_ RTC_ACQUIRED_BEFORE(_crit_capture);
    rtc::CriticalSection *crit_capture_;
    webrtc::GainControl *limiter_;
    webrtc::AudioBuffer *audio_buffer_;
    size_t num_channels_, sample_10ms_;
};

int Aproc_Limiter_Create(AprocLimiter **handle,
                         AprocLimiterParam *parameter)
{
    if (NULL == handle
        || NULL == parameter)
    {
        return -1;
    }

    int return_code;
    AprocLimiter *handle_tmp = NULL;
    do
    {
        handle_tmp =
            (AprocLimiter *)malloc(sizeof(AprocLimiter));
        if (NULL == handle_tmp)
        {
            return_code = -1;
            break;
        }

        handle_tmp->crit_render_ = new rtc::CriticalSection();
        handle_tmp->crit_capture_ = new rtc::CriticalSection();
        handle_tmp->limiter_ =
            new webrtc::GainControlImpl(handle_tmp->crit_render_, handle_tmp->crit_capture_);
        handle_tmp->limiter_->Initialize(parameter->num_channels_, parameter->fs_hz_);
        handle_tmp->limiter_->set_mode(webrtc::GainControl::kFixedDigital);
        // We smoothly limit the mixed frame to -7 dbFS. -6 would correspond to the
        // divide-by-2 but -7 is used instead to give a bit of headroom since the
        // AGC is not a hard limiter.
        handle_tmp->limiter_->set_target_level_dbfs(7);
        handle_tmp->limiter_->set_compression_gain_db(0);
        handle_tmp->limiter_->enable_limiter(true);
        handle_tmp->limiter_->Enable(true);
        // handle_tmp->limiter_.SetGain(0.f);
        handle_tmp->audio_buffer_ =
            new webrtc::AudioBuffer(parameter->fs_hz_, parameter->num_channels_,
                                    parameter->fs_hz_, parameter->num_channels_,
                                    parameter->fs_hz_, parameter->num_channels_);
        handle_tmp->audio_buffer_->set_activity(webrtc::AudioFrame::kVadActive);
        handle_tmp->num_channels_ = parameter->num_channels_;
        handle_tmp->sample_10ms_ =
            parameter->fs_hz_ * parameter->num_channels_ / (1000 / 10);

        return_code = 0;
    }
    while (0);

    *handle = handle_tmp;

    return return_code;
}

int Aproc_Limiter_Destroy(AprocLimiter *handle)
{
    if (NULL == handle)
    {
        return -1;
    }

    int return_code;
    do
    {
        delete handle->audio_buffer_;
        delete handle->limiter_;
        delete handle->crit_capture_;
        delete handle->crit_render_;
        free(handle);

        return_code = 0;
    }
    while (0);

    return return_code;
}

int Aproc_Limiter_Process(AprocLimiter *handle,
                          const int16_t *input, size_t input_length,
                          int16_t *output, size_t *output_length)
{
    if (NULL == handle
        || NULL == input || 0 == input_length
        || NULL == output || NULL == output_length || 0 == *output_length)
    {
        return -1;
    }

    int return_code;
    do
    {
        // check |input_length|
        if (input_length % handle->sample_10ms_ != 0)
        {
            printf("limiter: (%zu) mod (%zu) != (0) cannot process, skip\n", input_length, handle->sample_10ms_);
            return_code = -1;
            break;
        }
        // check |output_length|
        if (input_length > *output_length)
        {
            printf("limiter: (%zu) > (%zu) will cause overflow, skip\n", input_length, *output_length);
            return_code = -1;
            break;
        }

        // agc can only process 10ms sample, so loop
        size_t offset = 0;
        do
        {
            int16_t *const *deinterleaved = handle->audio_buffer_->channels();
            if (1)
                // Downmix and deinterleave simultaneously.
                webrtc::DownmixInterleavedToMono(input + offset, handle->sample_10ms_, handle->num_channels_, deinterleaved[0]);
            else
                webrtc::Deinterleave(input + offset, handle->sample_10ms_, handle->num_channels_, deinterleaved);

            // limit
            handle->audio_buffer_->SplitIntoFrequencyBands();
            // handle->audio_buffer_->CopyLowPassToReference();

            handle->limiter_->AnalyzeCaptureAudio(handle->audio_buffer_);
            handle->limiter_->ProcessCaptureAudio(handle->audio_buffer_);

            handle->audio_buffer_->MergeFrequencyBands();

            if (0)
                webrtc::Interleave(deinterleaved, handle->sample_10ms_, handle->num_channels_, output + offset);
            else
                webrtc::UpmixMonoToInterleaved(deinterleaved[0], handle->sample_10ms_, handle->num_channels_, output + offset);

            offset += handle->sample_10ms_;
            // printf("current offset %zu\n", offset);
        }
        while (offset < input_length);

        *output_length = input_length;

        return_code = 0;
    }
    while (0);

    return return_code;
}

// header
#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct AprocResampler AprocResampler;
typedef struct AprocResamplerParam AprocResamplerParam;

int Aproc_Resampler_Create(AprocResampler **handle,
                           AprocResamplerParam *parameter
                          );
int Aproc_Resampler_Destroy(AprocResampler *handle
                           );
int Aproc_Resampler_Process(AprocResampler *handle,
                            const int16_t *input, size_t input_length,
                            int16_t *output, size_t *output_length
                           );

struct AprocResamplerParam
{
    int fs_hz_in_;
    int fs_hz_out_;
    int num_channels_;
};

#if defined(__cplusplus)
};
#endif

// source
#include "common_audio/resampler/include/resampler.h"
// #include "common_audio/resampler/include/push_resampler.h"

struct AprocResampler
{
    webrtc::Resampler *resampler_;
    // webrtc::PushResampler<int16_t> *resampler_;
    size_t ratio_; // Q8
};

int Aproc_Resampler_Create(AprocResampler **handle,
                           AprocResamplerParam *parameter)
{
    if (NULL == handle
        || NULL == parameter)
    {
        return -1;
    }

    int return_code;
    AprocResampler *handle_tmp = NULL;
    do
    {
        handle_tmp =
            (AprocResampler *)malloc(sizeof(AprocResampler));
        if (NULL == handle_tmp)
        {
            return_code = -1;
            break;
        }

        handle_tmp->resampler_ =
            new webrtc::Resampler(parameter->fs_hz_in_,
                                  parameter->fs_hz_out_,
                                  parameter->num_channels_);
        // handle_tmp->resampler_ =
        //     new webrtc::PushResampler<int16_t>();
        // if (NULL == handle_tmp->resampler_)
        // {
        //     free(handle_tmp);
        //     handle_tmp = NULL;
        //     return_code = -1;
        //     break;
        // }
        // handle_tmp->resampler_->InitializeIfNeeded(
        //     parameter->fs_hz_in_,
        //     parameter->fs_hz_out_,
        //     parameter->num_channels_);

        handle_tmp->ratio_ = (parameter->fs_hz_out_ << 8) / parameter->fs_hz_in_;

        return_code = 0;
    }
    while (0);

    *handle = handle_tmp;

    return return_code;
}

int Aproc_Resampler_Destroy(AprocResampler *handle)
{
    if (NULL == handle)
    {
        return -1;
    }

    int return_code;
    do
    {
        delete handle->resampler_;
        free(handle);

        return_code = 0;
    }
    while (0);

    return return_code;
}

int Aproc_Resampler_Process(AprocResampler *handle,
                            const int16_t *input, size_t input_length,
                            int16_t *output, size_t *output_length)
{
    if (NULL == handle
        || NULL == input || 0 == input_length
        || NULL == output || NULL == output_length || 0 == *output_length)
    {
        return -1;
    }

    int return_code;
    do
    {
        // check |output_length|
        if ((handle->ratio_ * input_length) > (*output_length << 8))
        {
            printf("resampler: (%zu) > (%zu) will cause overflow, skip\n", (handle->ratio_ * input_length), (*output_length << 8));
            return_code = -1;
            break;
        }

        // resample
        handle->resampler_->Push(input, input_length,
                                 output, *output_length,
                                 *output_length);
        // *output_length = handle->resampler_->Resample(
        //                      input, input_length,
        //                      output, *output_length);

        return_code = 0;
    }
    while (0);

    return return_code;
}

// header
#if defined(__cplusplus)
extern "C"
{
#endif

typedef struct AprocNetEQ AprocNetEQ;
typedef struct AprocNetEQParam AprocNetEQParam;

int Aproc_NetEQ_Create(AprocNetEQ **handle,
                       AprocNetEQParam *parameter
                      );
int Aproc_NetEQ_Destroy(AprocNetEQ *handle
                       );
int Aproc_NetEQ_Process(AprocNetEQ *handle,
                        const int16_t *input, size_t input_length,
                        int16_t *output, size_t *output_length
                       );

struct AprocNetEQParam
{
    int fs_hz_;
    int num_channels_;
    void *buffer_handle_; // |sync_buffer_|
    size_t (*Buffer_Size)(void *); // used size
    void (*Buffer_ReadFromEnd)(void *, int16_t *, size_t); // borrow
};

#if defined(__cplusplus)
};
#endif

// source
#include "common_audio/signal_processing/include/signal_processing_library.h"
#include "modules/audio_coding/neteq/normal.h"
#include "modules/audio_coding/neteq/accelerate.h"
#include "modules/audio_coding/neteq/preemptive_expand.h"
#include "modules/audio_coding/neteq/buffer_level_filter.h"
#include "modules/audio_coding/neteq/delay_manager.h"
#include "modules/audio_coding/neteq/delay_peak_detector.h"
#include "modules/audio_coding/neteq/tick_timer.h"

struct AprocNetEQ
{
    // buffer_level
    webrtc::TickTimer *tick_timer_;
    std::unique_ptr<webrtc::TickTimer::Countdown> timescale_countdown_;
    webrtc::DelayPeakDetector *delay_peak_detector_;
    std::unique_ptr<webrtc::DelayManager> delay_manager_;
    webrtc::BufferLevelFilter *buffer_level_filter_;
    // time_stretch
    webrtc::Normal *normal_;
    webrtc::Accelerate *accelerate_;
    webrtc::PreemptiveExpand *preemptive_expand_;
    webrtc::AudioMultiVector *algorithm_buffer_;
    // borrow
    void *buffer_handle_; // |sync_buffer_|
    size_t (*Buffer_Size)(void *); // used size
    void (*Buffer_ReadFromEnd)(void *, int16_t *, size_t); // borrow
    int16_t *buffer_tmp_; // |required_samples_| size
    // parameter
    size_t num_channels_;
    size_t fs_hz_;
    size_t ts_interval_;
    size_t required_samples_;
    // record
    size_t time_stretched_samples_;
    bool prev_time_scale_;
    uint16_t main_sequence_number_;
    uint32_t main_timestamp_;
    // cache
    size_t fs_mult_;
    size_t ts_increment_;
};

int Aproc_NetEQ_Create(AprocNetEQ **handle,
                       AprocNetEQParam *parameter)
{
    WebRtcSpl_Init(); // *MUST* be called, multiple calls is acceptable

    if (NULL == handle
        || NULL == parameter)
    {
        return -1;
    }

    int return_code;
    AprocNetEQ *handle_tmp = NULL;
    do
    {
        if (/* NULL == parameter->buffer_handle_
            || */ NULL == parameter->Buffer_Size
            || NULL == parameter->Buffer_ReadFromEnd)
        {
            return_code = -1;
            break;
        }

        handle_tmp =
            new AprocNetEQ; // when contains unique_ptr, must call new instead of delete
        if (NULL == handle_tmp)
        {
            return_code = -1;
            break;
        }

        handle_tmp->num_channels_ = parameter->num_channels_;
        handle_tmp->fs_hz_ = parameter->fs_hz_;
        handle_tmp->fs_mult_ = handle_tmp->fs_hz_ / 8000;
        handle_tmp->required_samples_ = 240 * handle_tmp->fs_mult_;  // Must have 30 ms.
        handle_tmp->ts_increment_ = 80 * handle_tmp->fs_mult_; // 10 ms
        handle_tmp->ts_interval_ = 10; // TODO find a good value
        handle_tmp->buffer_handle_ = parameter->buffer_handle_;
        handle_tmp->Buffer_Size = parameter->Buffer_Size;
        handle_tmp->Buffer_ReadFromEnd = parameter->Buffer_ReadFromEnd;
        handle_tmp->buffer_tmp_ = (int16_t *)malloc(handle_tmp->required_samples_ * sizeof(int16_t));

        handle_tmp->time_stretched_samples_ = 0;
        handle_tmp->prev_time_scale_ = false;
        handle_tmp->main_sequence_number_ = 0x1234; // whatever
        handle_tmp->main_timestamp_ = 0x12345678; // whatever

        /* static */ const int kMaxNumberOfPackets = 20; // seems whatever
        handle_tmp->tick_timer_ =
            new webrtc::TickTimer;
        handle_tmp->timescale_countdown_ =
            handle_tmp->tick_timer_->GetNewCountdown(handle_tmp->ts_interval_ + 1);
        handle_tmp->delay_peak_detector_ =
            new webrtc::DelayPeakDetector(handle_tmp->tick_timer_, false);
        handle_tmp->delay_manager_ =
            webrtc::DelayManager::Create(kMaxNumberOfPackets, handle_tmp->delay_peak_detector_, handle_tmp->tick_timer_);
        handle_tmp->delay_manager_->set_streaming_mode(true);
        // TODO what is the proper |frame_size_ms|
        /* const */ size_t frame_size_ms = (1000 * handle_tmp->ts_increment_) / handle_tmp->fs_hz_;
        handle_tmp->delay_manager_->SetPacketAudioLength(frame_size_ms);
        handle_tmp->delay_manager_->Update(handle_tmp->main_sequence_number_, handle_tmp->main_timestamp_, handle_tmp->fs_hz_);

        handle_tmp->buffer_level_filter_ =
            new webrtc::BufferLevelFilter;
        handle_tmp->buffer_level_filter_->SetTargetBufferLevel(handle_tmp->delay_manager_->base_target_level());

        /* const */ size_t overlap_samples = 5 * handle_tmp->fs_mult_;
        handle_tmp->normal_ =
            new webrtc::Normal(handle_tmp->fs_hz_); // no need for |num_channels_|
        handle_tmp->accelerate_ =
            new webrtc::Accelerate(handle_tmp->fs_hz_, handle_tmp->num_channels_);
        // TODO proper |overlap_samples|
        handle_tmp->preemptive_expand_ =
            new webrtc::PreemptiveExpand(handle_tmp->fs_hz_, handle_tmp->num_channels_, overlap_samples);
        handle_tmp->algorithm_buffer_ =
            new webrtc::AudioMultiVector(handle_tmp->num_channels_);
        // TODO check pointers

        return_code = 0;
    }
    while (0);

    *handle = handle_tmp;

    return return_code;
}

int Aproc_NetEQ_Destroy(AprocNetEQ *handle)
{
    if (NULL == handle)
    {
        return -1;
    }

    int return_code;
    do
    {
        delete handle->algorithm_buffer_;
        delete handle->preemptive_expand_;
        delete handle->accelerate_;
        delete handle->normal_;
        delete handle->buffer_level_filter_;
        // handle->delay_manager_ = nullptr;
        delete handle->delay_peak_detector_;
        handle->timescale_countdown_ = nullptr; // delete std::unique_ptr
        delete handle->tick_timer_;
        free(handle->buffer_tmp_);
        delete handle;

        return_code = 0;
    }
    while (0);

    return return_code;
}

static inline bool IsTimescaleAllowed(AprocNetEQ *handle)
{
    return (!handle->timescale_countdown_ || handle->timescale_countdown_->Finished());
}
static inline bool IsBorrowAllowed(AprocNetEQ *handle, size_t input_length)
{
    // In order to do an accelerate/preemptive_expand we need at least 30 ms of audio data.
    return (handle->Buffer_Size(handle->buffer_handle_) + input_length >= handle->required_samples_);
}
static inline bool BorrowIfNeeded(AprocNetEQ *handle, const int16_t *input, size_t input_length)
{
    if (input_length < handle->required_samples_)
    {
        handle->Buffer_ReadFromEnd(handle->buffer_handle_, handle->buffer_tmp_, handle->required_samples_ - input_length);
        memmove(handle->buffer_tmp_ + handle->required_samples_ - input_length, input, input_length * sizeof(int16_t));

        return true;
    }
    else
    {
        return false;
    }
}
static inline int DoNormal(AprocNetEQ *handle,
                           const int16_t *input, size_t input_length,
                           int16_t *output, size_t *output_length)
{
    size_t before_samples, after_samples;
    before_samples = handle->algorithm_buffer_->Size();
    int return_code =
        handle->normal_->Process(input, input_length, handle->algorithm_buffer_);
    after_samples = handle->algorithm_buffer_->Size();
    // printf("kNormal: return_code(%d), before_samples(%zu), after_samples(%zu)\n",
    //        return_code, before_samples, after_samples);

    *output_length = after_samples;
    handle->algorithm_buffer_->ReadInterleaved(*output_length, output);
    handle->algorithm_buffer_->Clear();
    handle->time_stretched_samples_ = 0;
    handle->prev_time_scale_ = false;

    return return_code;
}
static inline int AccelerateHelper(AprocNetEQ *handle, bool fast_accelerate,
                                   const int16_t *input, size_t input_length,
                                   int16_t *output, size_t *output_length
                                  )
{
    size_t before_samples, after_samples;
    before_samples = handle->algorithm_buffer_->Size();
    size_t samples_removed;
    webrtc::Accelerate::ReturnCodes return_code;
    if (BorrowIfNeeded(handle, input, input_length))
    {
        return_code =
            handle->accelerate_->Process(handle->buffer_tmp_, handle->required_samples_, fast_accelerate,
                                         handle->algorithm_buffer_, &samples_removed);
    }
    else
    {
        return_code =
            handle->accelerate_->Process(input, input_length, fast_accelerate,
                                         handle->algorithm_buffer_, &samples_removed);
    }
    after_samples = handle->algorithm_buffer_->Size();
    // printf("k%sAccelerate: return_code(%d), samples_removed(%zu), before_samples(%zu), after_samples(%zu)\n",
    //        fast_accelerate ? "Fast" : "", return_code, samples_removed, before_samples, after_samples);

    *output_length = after_samples;
    handle->algorithm_buffer_->ReadInterleaved(*output_length, output);
    handle->algorithm_buffer_->Clear();
    handle->time_stretched_samples_ = samples_removed;
    handle->prev_time_scale_ = true;

    return (webrtc::Accelerate::kError == return_code ? -1 : 0);
}
static inline int DoAccelerate(AprocNetEQ *handle,
                               const int16_t *input, size_t input_length,
                               int16_t *output, size_t *output_length
                              )
{
    return AccelerateHelper(handle, false, input, input_length, output, output_length);
}
static inline int DoFastAccelerate(AprocNetEQ *handle,
                                   const int16_t *input, size_t input_length,
                                   int16_t *output, size_t *output_length
                                  )
{
    return AccelerateHelper(handle, true, input, input_length, output, output_length);
}
static inline int DoPreemptiveExpand(AprocNetEQ *handle,
                                     const int16_t *input, size_t input_length,
                                     int16_t *output, size_t *output_length
                                    )
{
    size_t before_samples, after_samples;
    before_samples = handle->algorithm_buffer_->Size();
    size_t samples_added;
    webrtc::PreemptiveExpand::ReturnCodes return_code;
    if (BorrowIfNeeded(handle, input, input_length))
    {
        // TODO proper |old_data_len|
        return_code =
            handle->preemptive_expand_->Process(handle->buffer_tmp_, handle->required_samples_, 0,
                                                handle->algorithm_buffer_, &samples_added);
    }
    else
    {
        return_code =
            handle->preemptive_expand_->Process(input, input_length, 0,
                                                handle->algorithm_buffer_, &samples_added);
    }
    after_samples = handle->algorithm_buffer_->Size();
    // printf("kPreemptiveExpand: return_code(%d), samples_added(%zu), before_samples(%zu), after_samples(%zu)\n",
    //        return_code, samples_added, before_samples, after_samples);

    *output_length = after_samples;
    handle->algorithm_buffer_->ReadInterleaved(*output_length, output);
    handle->algorithm_buffer_->Clear();
    handle->time_stretched_samples_ = 0 - samples_added; // negative
    handle->prev_time_scale_ = true;

    return (webrtc::PreemptiveExpand::kError == return_code ? -1 : 0);
}

int Aproc_NetEQ_Process(AprocNetEQ *handle,
                        const int16_t *input, size_t input_length,
                        int16_t *output, size_t *output_length)
{
    if (NULL == handle
        || NULL == input || 0 == input_length
        || NULL == output || NULL == output_length || 0 == *output_length)
    {
        return -1;
    }

    int return_code;
    do
    {
        // IncreaseTime
        for (size_t t = 0; t < input_length; t += handle->ts_increment_)
        {
            handle->tick_timer_->Increment();
        }

        // Decision (normal/accelerate/preemptive_expand)
        if (!IsBorrowAllowed(handle, input_length))
        {
            DoNormal(handle, input, input_length, output, output_length);
        }
        else
        {
            int low_limit, high_limit, current_level;
            handle->delay_manager_->BufferLimits(&low_limit, &high_limit);
            current_level = handle->buffer_level_filter_->filtered_current_level();
            if (current_level >= high_limit << 2)
            {
                DoFastAccelerate(handle, input, input_length, output, output_length);
            }
            else if (current_level >= high_limit)
            {
                if (!IsTimescaleAllowed(handle))
                {
                    DoNormal(handle, input, input_length, output, output_length);
                }
                else
                {
                    DoAccelerate(handle, input, input_length, output, output_length);
                }
            }
            else if (current_level < low_limit)
            {
                if (!IsTimescaleAllowed(handle))
                {
                    DoNormal(handle, input, input_length, output, output_length);
                }
                else
                {
                    DoPreemptiveExpand(handle, input, input_length, output, output_length);
                }
            }
            else
            {
                DoNormal(handle, input, input_length, output, output_length);
            }
            // update |timescale_countdown_|
            if (handle->prev_time_scale_)
            {
                handle->timescale_countdown_ =
                    handle->tick_timer_->GetNewCountdown(handle->ts_interval_);
            }
        }

        // InsertNextPacket
        handle->main_sequence_number_ += 1;
        handle->main_timestamp_ += handle->ts_increment_;
        handle->delay_manager_->Update(handle->main_sequence_number_, handle->main_timestamp_, handle->fs_hz_);
        handle->buffer_level_filter_->SetTargetBufferLevel(handle->delay_manager_->base_target_level());
        size_t buffer_size_packets = handle->Buffer_Size(handle->buffer_handle_) / handle->ts_increment_;
        handle->buffer_level_filter_->Update(buffer_size_packets, handle->time_stretched_samples_, handle->ts_increment_);

        return_code = 0;
    }
    while (0);

    return return_code;
}


// source
struct AudioWebrtcProc
{
    AprocResampler   *resampler_;
    AprocNetEQ       *neteq_;
    AprocLimiter     *limiter_;

    AudioWebrtcProcParam parameter_;
};

int Audio_Webrtc_Proc_Create(AudioWebrtcProc **handle,
                             AudioWebrtcProcParam *parameter)
{
    if (NULL == handle
        || NULL == parameter)
    {
        return -1;
    }

    int return_code;
    AudioWebrtcProc *handle_tmp = NULL;
    int fs_hz_tmp = parameter->fs_hz_in_;
    do
    {
        handle_tmp =
            (AudioWebrtcProc *)malloc(sizeof(AudioWebrtcProc));

        if (parameter->enable_limiter_)
        {
            AprocLimiterParam parameter_tmp;
            parameter_tmp.fs_hz_ = fs_hz_tmp;
            parameter_tmp.num_channels_ = parameter->num_channels_;
            Aproc_Limiter_Create(&handle_tmp->limiter_, &parameter_tmp);
        }
        if (parameter->enable_resampler_)
        {
            AprocResamplerParam parameter_tmp;
            parameter_tmp.fs_hz_in_ = fs_hz_tmp;
            parameter_tmp.fs_hz_out_ = parameter->fs_hz_out_;
            parameter_tmp.num_channels_ = parameter->num_channels_;
            Aproc_Resampler_Create(&handle_tmp->resampler_, &parameter_tmp);

            fs_hz_tmp = parameter_tmp.fs_hz_out_;
        }
        if (parameter->enable_neteq_)
        {
            // DONE check |fs_hz_in_| or |fs_hz_out_|
            AprocNetEQParam parameter_tmp;
            parameter_tmp.fs_hz_ = fs_hz_tmp; // parameter->fs_hz_out_; // using |resampler_| output
            parameter_tmp.num_channels_ = parameter->num_channels_;
            parameter_tmp.buffer_handle_ = parameter->buffer_handle_;
            parameter_tmp.Buffer_Size = parameter->Buffer_Size;
            parameter_tmp.Buffer_ReadFromEnd = parameter->Buffer_ReadFromEnd;
            Aproc_NetEQ_Create(&handle_tmp->neteq_, &parameter_tmp);
        }

        handle_tmp->parameter_ = *parameter;

        return_code = 0;
    }
    while (0);

    *handle = handle_tmp;

    return return_code;
}

int Audio_Webrtc_Proc_Destroy(AudioWebrtcProc *handle)
{
    if (NULL == handle)
    {
        return -1;
    }

    int return_code;
    do
    {
        if (handle->parameter_.enable_neteq_)
        {
            Aproc_NetEQ_Destroy(handle->neteq_);
        }
        if (handle->parameter_.enable_resampler_)
        {
            Aproc_Resampler_Destroy(handle->resampler_);
        }
        if (handle->parameter_.enable_limiter_)
        {
            Aproc_Limiter_Destroy(handle->limiter_);
        }
        free(handle);

        return_code = 0;
    }
    while (0);

    return return_code;
}

int Audio_Webrtc_Proc_Process(AudioWebrtcProc *handle,
                              const int16_t *input, size_t input_length,
                              int16_t *output, size_t *output_length)
{
    if (NULL == handle
        || NULL == input || 0 == input_length
        || NULL == output || NULL == output_length || 0 == *output_length)
    {
        return -1;
    }

    int return_code;
    int16_t *input_tmp = (int16_t *)input;
    size_t input_length_tmp = input_length, output_length_tmp;
    do
    {
        // TODO check return value, if not sucess, do not change |input_tmp|
        if (handle->parameter_.enable_limiter_)
        {
            output_length_tmp = *output_length;
            Aproc_Limiter_Process(handle->limiter_,
                                  input_tmp, input_length_tmp,
                                  output, &output_length_tmp);
            input_tmp = output;
            input_length_tmp = output_length_tmp;
        }
        if (handle->parameter_.enable_resampler_)
        {
            output_length_tmp = *output_length;
            Aproc_Resampler_Process(handle->resampler_,
                                    input_tmp, input_length_tmp,
                                    output, &output_length_tmp);
            input_tmp = output;
            input_length_tmp = output_length_tmp;
        }
        if (handle->parameter_.enable_neteq_)
        {
            output_length_tmp = *output_length;
            Aproc_NetEQ_Process(handle->neteq_,
                                input_tmp, input_length_tmp,
                                output, &output_length_tmp);
            input_tmp = output;
            input_length_tmp = output_length_tmp;
        }
        // TODO if nothing is enable, just copy input to output
        *output_length = output_length_tmp;

        return_code = 0;
    }
    while (0);

    return return_code;
}
