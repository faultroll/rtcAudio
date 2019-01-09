
#define WEBRTC_POSIX
#include "modules/audio_processing/agc/gain_control_impl.h"
#include <stdint.h>
#include <stdio.h>
#include <memory> // std::unique_ptr
#include "modules/include/audio_frame.h"
#include "modules/audio_processing/audio_buffer.h"

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

    int read_size = 0;
    webrtc::AudioFrame audio_frame;
    std::unique_ptr<webrtc::AudioBuffer> audio_buffer;
    audio_frame.UpdateFrame(0, nullptr,
                            kSample10Ms / kNumChannels, kSampleRateHz,
                            webrtc::AudioFrame::kNormalSpeech, webrtc::AudioFrame::kVadActive,
                            kNumChannels);
    audio_buffer.reset(
        new webrtc::AudioBuffer(kSampleRateHz, kNumChannels,
                                kSampleRateHz, kNumChannels,
                                kSampleRateHz, kNumChannels));

    do
    {
        read_size = fread(audio_frame.mutable_data(), sizeof(int16_t), kSample10Ms, file_in);

        audio_buffer->DeinterleaveFrom(&audio_frame);
        audio_buffer->SplitIntoFrequencyBands();
        audio_buffer->CopyLowPassToReference();

        limiter->AnalyzeCaptureAudio(audio_buffer.get());
        limiter->ProcessCaptureAudio(audio_buffer.get());

        audio_buffer->MergeFrequencyBands();
        audio_buffer->InterleaveTo(&audio_frame, true);

        fwrite(audio_frame.data(), sizeof(int16_t), kSample10Ms, file_out);
    }
    while (read_size);

    fclose(file_in);
    fclose(file_out);

    return 0;
}

