
#define WEBRTC_POSIX
#include "modules/audio_processing/agc/gain_control_impl.h"
#include <stdint.h>
#include <stdio.h>
#include <memory> // std::unique_ptr
#include "modules/include/audio_frame.h"
#include "common_audio/include/audio_util.h"
#include "modules/audio_processing/audio_buffer.h"

#define USING_AUDIO_FRAME

int main(void)
{
    // bitwidth is always 16-bit
    static const int kSampleRateHz = 8000,
                     kNumChannels = 1,
                     kSample10Ms = kSampleRateHz * kNumChannels / (1000 / 10);

    FILE *file_in = nullptr, *file_out = nullptr;
    file_in = fopen("noise_8000.pcm", "rb");
    file_out = fopen("agc_8000_16_1.pcm", "wb");

    rtc::CriticalSection crit_render, crit_capture;
    std::unique_ptr<webrtc::GainControl> limiter;
    limiter.reset(
        new webrtc::GainControlImpl(&crit_render, &crit_capture));
    limiter->Initialize(kNumChannels, kSampleRateHz);
    limiter->set_mode(webrtc::GainControl::kFixedDigital);
    // We smoothly limit the mixed frame to -7 dbFS. -6 would correspond to the
    // divide-by-2 but -7 is used instead to give a bit of headroom since the
    // AGC is not a hard limiter.
    limiter->set_target_level_dbfs(7);
    limiter->set_compression_gain_db(0);
    limiter->enable_limiter(true);
    limiter->Enable(true);
    // limiter.SetGain(0.f);

    std::unique_ptr<webrtc::AudioBuffer> audio_buffer;
    audio_buffer.reset(
        new webrtc::AudioBuffer(kSampleRateHz, kNumChannels,
                                kSampleRateHz, kNumChannels,
                                kSampleRateHz, kNumChannels));
    audio_buffer->set_activity(webrtc::AudioFrame::kVadActive);

#ifdef USING_AUDIO_FRAME
    webrtc::AudioFrame audio_frame;
    audio_frame.UpdateFrame(0, nullptr,
                            kSample10Ms / kNumChannels, kSampleRateHz,
                            webrtc::AudioFrame::kNormalSpeech, webrtc::AudioFrame::kVadActive,
                            kNumChannels);
#else
    int16_t *const *deinterleaved = audio_buffer->channels();
    int16_t *buffer_in = new int16_t[kSample10Ms];
    int16_t *buffer_out = new int16_t[kSample10Ms];
#endif

    int read_size = 0;
    do
    {
#ifdef USING_AUDIO_FRAME
        read_size = fread(audio_frame.mutable_data(), sizeof(int16_t), kSample10Ms, file_in);
        audio_buffer->DeinterleaveFrom(&audio_frame);
#else
        read_size = fread(buffer_in, sizeof(int16_t), kSample10Ms, file_in);
        if (1)
            // Downmix and deinterleave simultaneously.
            webrtc::DownmixInterleavedToMono(buffer_in, kSample10Ms, kNumChannels, deinterleaved[0]);
        else
            webrtc::Deinterleave(buffer_in, kSample10Ms, kNumChannels, deinterleaved);
#endif

        audio_buffer->SplitIntoFrequencyBands();
        audio_buffer->CopyLowPassToReference();

        limiter->AnalyzeCaptureAudio(audio_buffer.get());
        limiter->ProcessCaptureAudio(audio_buffer.get());

        audio_buffer->MergeFrequencyBands();

#ifdef USING_AUDIO_FRAME
        audio_buffer->InterleaveTo(&audio_frame, true);
        fwrite(audio_frame.data(), sizeof(int16_t), kSample10Ms, file_out);
#else
        if (0)
            webrtc::Interleave(deinterleaved, kSample10Ms, kNumChannels, buffer_out);
        else
            webrtc::UpmixMonoToInterleaved(deinterleaved[0], kSample10Ms, kNumChannels, buffer_out);
        fwrite(buffer_out, sizeof(int16_t), kSample10Ms, file_out);
#endif
    }
    while (read_size);

#ifdef USING_AUDIO_FRAME
#else
    delete[] buffer_in;
    delete[] buffer_out;
#endif

    fclose(file_in);
    fclose(file_out);

    return 0;
}

