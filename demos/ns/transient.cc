
#include "modules/audio_processing/transient/transient_suppressor.h"
#include <stdint.h>
#include <stdio.h>
#include <memory> // std::unique_ptr
#include "modules/include/audio_frame.h"
#include "modules/audio_processing/audio_buffer.h"

int main(void)
{
    // bitwidth is always 16-bit
    static const int kSampleRateHz = 32000,
                     kNumChannels = 1,
                     kSample10Ms = kSampleRateHz * kNumChannels / (1000 / 10);

    FILE *file_in = nullptr, *file_out = nullptr;
    file_in = fopen("noise_32000.pcm", "rb");
    file_out = fopen("transient_32000_16_1.pcm", "wb");

    std::unique_ptr<webrtc::TransientSuppressor> suppressor;
    suppressor.reset(
        new webrtc::TransientSuppressor());
    suppressor->Initialize(kSampleRateHz, kSampleRateHz, kNumChannels);

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
        // audio_buffer->CopyLowPassToReference();

        suppressor->Suppress(
            audio_buffer->channels_f()[0],
            audio_buffer->num_frames(),
            audio_buffer->num_channels(),
            audio_buffer->split_bands_const_f(0)[webrtc::Band::kBand0To8kHz],
            audio_buffer->num_frames_per_band(),
            NULL, 0,
            /* _agc_manager->voice_probability() */1.f,
            /* key_pressed */true);

        audio_buffer->MergeFrequencyBands();
        audio_buffer->InterleaveTo(&audio_frame, true);

        fwrite(audio_frame.data(), sizeof(int16_t), kSample10Ms, file_out);
    }
    while (read_size);

    fclose(file_in);
    fclose(file_out);

    return 0;
}

