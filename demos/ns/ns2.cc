
#define WEBRTC_POSIX
#include "modules/audio_processing/ns2/noise_suppressor.h"
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
    file_out = fopen("ns2_32000_16_1.pcm", "wb");

    webrtc::NsConfig config;
    config.target_level = webrtc::NsConfig::SuppressionLevel::k6dB;
    std::unique_ptr<webrtc::NoiseSuppressor> suppressor;
    suppressor.reset(
        new webrtc::NoiseSuppressor(config, kSampleRateHz, kNumChannels));

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

        suppressor->Analyze(*audio_buffer.get());
        suppressor->Process(audio_buffer.get());

        audio_buffer->MergeFrequencyBands();
        audio_buffer->InterleaveTo(&audio_frame, true);

        fwrite(audio_frame.data(), sizeof(int16_t), kSample10Ms, file_out);
    }
    while (read_size);

    fclose(file_in);
    fclose(file_out);

    return 0;
}

