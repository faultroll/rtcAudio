#include <stdio.h>
#include <string.h>
#include "modules/audio_mixer/audio_mixer.h"
// #include "modules/audio_mixer/audio_mixer_impl.h"
// #include "modules/audio_mixer/default_output_rate_calculator.h"

static int iSampleRate = 48000;
static int iChannels = 2;
static int iBit = 16;

class AudioSrc : public webrtc::AudioMixer::Source
{
public:
    AudioSrc(int i): m_isrc(i)
    {

    }
    ~AudioSrc()
    {

    }

    virtual AudioFrameInfo GetAudioFrameWithInfo(int sample_rate_hz,
            webrtc::AudioFrame *audio_frame)
    {
        audio_frame->CopyFrom(m_audioFrame);

        return AudioFrameInfo::kNormal;
    }

    virtual int Ssrc() const
    {
        return m_isrc;
    }

    virtual int PreferredSampleRate() const
    {
        return iSampleRate;
    }

    void filldata(short *pdata, int iSample)
    {
        m_audioFrame.id_ = m_isrc;
        m_audioFrame.num_channels_ = iChannels;
        m_audioFrame.samples_per_channel_ = iSampleRate / (1000 / 10);
        m_audioFrame.sample_rate_hz_ = iSampleRate;
        m_audioFrame.speech_type_ = webrtc::AudioFrame::kNormalSpeech;
        m_audioFrame.vad_activity_ = webrtc::AudioFrame::kVadActive;

        memcpy(m_audioFrame.mutable_data(), pdata, iSample * sizeof(short));

    }
    webrtc::AudioFrame m_audioFrame;
    int m_isrc;
};

int main(int argc, char *argv[])
{
    const int i10msSz = iSampleRate * iChannels * (iBit / 8)
                        / (1000 / 10) / sizeof(short); // 10ms Sample, short

    FILE *pSrc1 = nullptr, *pSrc2 = nullptr, *pSrc3 = nullptr,
          *pSrc4 = nullptr, *pSrc5 = nullptr, *pSrc6 = nullptr;
    FILE *pOut = nullptr;

    pSrc1 = fopen("demos/samples/1_48000_16_2.pcm", "rb");
    pSrc2 = fopen("demos/samples/2_48000_16_2.pcm", "rb");
    pSrc3 = fopen("demos/samples/3_48000_16_2.pcm", "rb");
    pSrc4 = fopen("demos/samples/4_48000_16_2.pcm", "rb");
    pSrc5 = fopen("demos/samples/5_48000_16_2.pcm", "rb");
    pSrc6 = fopen("demos/samples/6_48000_16_2.pcm", "rb");
    pOut = fopen("mixed_48000_16_2.pcm", "wb");

    short *pBuf1 = new short[i10msSz];
    short *pBuf2 = new short[i10msSz];
    short *pBuf3 = new short[i10msSz];
    short *pBuf4 = new short[i10msSz];
    short *pBuf5 = new short[i10msSz];
    short *pBuf6 = new short[i10msSz];
    // rtc::scoped_refptr<webrtc::AudioMixerImpl> mixptr = webrtc::AudioMixerImpl::Create();
    /* struct test
    {
        rtc::scoped_refptr<webrtc::AudioMixerImpl> mixptr;
    } *test = new struct test;
    test->mixptr = webrtc::AudioMixerImpl::Create(); */
    rtc::scoped_refptr<webrtc::AudioMixer> mixptr = webrtc::AudioMixer::Create();

    AudioSrc *src1 = new AudioSrc(1);
    AudioSrc *src2 = new AudioSrc(2);
    AudioSrc *src3 = new AudioSrc(3);
    AudioSrc *src4 = new AudioSrc(4);
    AudioSrc *src5 = new AudioSrc(5);
    AudioSrc *src6 = new AudioSrc(6);
    webrtc::AudioFrame audioframe;

    mixptr->AddSource(src1);
    mixptr->AddSource(src2);
    mixptr->AddSource(src3);
    mixptr->AddSource(src4);
    mixptr->AddSource(src5);
    mixptr->AddSource(src6);
    // mixptr->RemoveSource(src1);

    int iR1 = 0, iR2 = 0, iR3 = 0, iR4 = 0, iR5 = 0, iR6 = 0;

    do
    {
        iR1 = fread(pBuf1, sizeof(short), i10msSz, pSrc1);
        iR2 = fread(pBuf2, sizeof(short), i10msSz, pSrc2);
        iR3 = fread(pBuf3, sizeof(short), i10msSz, pSrc3);
        iR4 = fread(pBuf4, sizeof(short), i10msSz, pSrc4);
        iR5 = fread(pBuf5, sizeof(short), i10msSz, pSrc5);
        iR6 = fread(pBuf6, sizeof(short), i10msSz, pSrc6);

        src1->filldata(pBuf1, iR1);
        src2->filldata(pBuf2, iR2);
        src3->filldata(pBuf3, iR3);
        src4->filldata(pBuf4, iR4);
        src5->filldata(pBuf5, iR5);
        src6->filldata(pBuf6, iR6);

        mixptr->Mix(iChannels, &audioframe);

        fwrite(audioframe.data(), sizeof(int16_t), iR1, pOut);
    }
    while (iR1 && iR2 && iR3 && iR4 && iR5 && iR6);

    delete src1;
    delete src2;
    delete src3;
    delete src4;
    delete src5;
    delete src6;
    // delete test; // cannot use free(test); it won't call ~xxx functions
    // delete mixptr; // cannot do this, scoped_refptr is auto deleted

    delete[] pBuf1;
    delete[] pBuf2;
    delete[] pBuf3;
    delete[] pBuf4;
    delete[] pBuf5;
    delete[] pBuf6;

    fclose(pSrc1);
    fclose(pSrc2);
    fclose(pSrc3);
    fclose(pSrc4);
    fclose(pSrc5);
    fclose(pSrc6);
    fclose(pOut);

    return 0;
}