
#include "common_audio/resampler/include/push_resampler.h"
#include <stdint.h>
#include <stdio.h>
#include <memory> // std::unique_ptr

int main(void)
{
    // bitwidth is always 16-bit
    static const int kSampleRateHzIn = 8000, kSampleRateHzOut = 32000,
                     kNumChannels = 1,
                     kSample10MsIn = kSampleRateHzIn * kNumChannels / (1000 / 10),
                     kSample10MsOut = kSampleRateHzOut * kNumChannels / (1000 / 10);

    FILE *file_in = nullptr, *file_out = nullptr;
    file_in = fopen("agc_8000_16_1.pcm", "rb");
    file_out = fopen("sinc_resampler_32000_16_1.pcm", "wb");

    std::unique_ptr<webrtc::PushResampler<int16_t>> resampler;
    resampler.reset(
        new webrtc::PushResampler<int16_t>());
    resampler->InitializeIfNeeded(
        kSampleRateHzIn,
        kSampleRateHzOut,
        kNumChannels);

    int read_size = 0;
    size_t resampled_size;
    int16_t *buffer_in = new int16_t[kSample10MsIn];
    int16_t *buffer_out = new int16_t[kSample10MsOut];

    do
    {
        read_size = fread(buffer_in, sizeof(int16_t), kSample10MsIn, file_in);

        resampled_size =
            resampler->Resample(buffer_in, kSample10MsIn,
                                buffer_out, kSample10MsOut);

        fwrite(buffer_out, sizeof(int16_t), kSample10MsOut, file_out);
    }
    while (read_size);

    delete[] buffer_in;
    delete[] buffer_out;

    fclose(file_in);
    fclose(file_out);

    return 0;
}

