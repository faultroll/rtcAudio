#include<stdio.h>
// #include"webrtc/api/audio/audio_mixer.h"
#include"webrtc/modules/audio_mixer/audio_mixer_impl.h"
// #include"webrtc/modules/audio_mixer/default_output_rate_calculator.h"

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

    void fillaudioframe(short *pdata, int iSample)
    {
        m_audioFrame.samples_per_channel_ = iSampleRate / (1000 / 10);
        m_audioFrame.sample_rate_hz_ = iSampleRate;
        m_audioFrame.id_ = 1;
        m_audioFrame.num_channels_ = iChannels;

        memcpy(m_audioFrame.data_, pdata, iSample * sizeof(short));

        m_audioFrame.vad_activity_ = webrtc::AudioFrame::kVadActive;
        m_audioFrame.speech_type_ = webrtc::AudioFrame::kNormalSpeech;
    }
    webrtc::AudioFrame m_audioFrame;
    int m_isrc;
};
int main(int argc, char *argv[])
{
    int i10msSz = iSampleRate * iChannels * (iBit / 8)
                  / (1000 / 10) / sizeof(short); // 10ms Sample, short

    FILE *pSrc1 = nullptr, *pSrc2 = nullptr, *pSrc3 = nullptr,
          *pSrc4 = nullptr, *pSrc5 = nullptr, *pSrc6 = nullptr;
    FILE *pOut = nullptr;

    pSrc1 = fopen("1_4800_16_2.pcm", "rb+");
    pSrc2 = fopen("2_4800_16_2.pcm", "rb+");
    pSrc3 = fopen("3_4800_16_2.pcm", "rb+");
    pSrc4 = fopen("4_4800_16_2.pcm", "rb+");
    pSrc5 = fopen("5_4800_16_2.pcm", "rb+");
    pSrc6 = fopen("6_4800_16_2.pcm", "rb+");
    pOut = fopen("mix.pcm", "wb+");

    short *pBuf1 = new short[i10msSz];
    short *pBuf2 = new short[i10msSz];
    short *pBuf3 = new short[i10msSz];
    short *pBuf4 = new short[i10msSz];
    short *pBuf5 = new short[i10msSz];
    short *pBuf6 = new short[i10msSz];
    // auto mixptr = webrtc::AudioMixerImpl::Create();
    struct test
    {
        rtc::scoped_refptr<webrtc::AudioMixerImpl> mixptr;
    } *test = new struct test;
    test->mixptr = webrtc::AudioMixerImpl::Create();

    AudioSrc *src1 = new AudioSrc(1);
    AudioSrc *src2 = new AudioSrc(2);
    AudioSrc *src3 = new AudioSrc(3);
    AudioSrc *src4 = new AudioSrc(4);
    AudioSrc *src5 = new AudioSrc(5);
    AudioSrc *src6 = new AudioSrc(6);
    webrtc::AudioFrame audioframe;

    int iR1 = fread(pBuf1, sizeof(short), i10msSz, pSrc1);
    int iR2 = fread(pBuf2, sizeof(short), i10msSz, pSrc2);
    int iR3 = fread(pBuf3, sizeof(short), i10msSz, pSrc3);
    int iR4 = fread(pBuf4, sizeof(short), i10msSz, pSrc4);
    int iR5 = fread(pBuf5, sizeof(short), i10msSz, pSrc5);
    int iR6 = fread(pBuf6, sizeof(short), i10msSz, pSrc6);

    src1->fillaudioframe(pBuf1, iR1);
    src2->fillaudioframe(pBuf2, iR2);
    src3->fillaudioframe(pBuf3, iR3);
    // src4->fillaudioframe(pBuf4, iR4);
    // src5->fillaudioframe(pBuf5, iR5);
    // src6->fillaudioframe(pBuf6, iR6);

    test->mixptr->AddSource(src1);
    test->mixptr->AddSource(src2);
    test->mixptr->AddSource(src3);
    // test->mixptr->AddSource(src4);
    // test->mixptr->AddSource(src5);
    // test->mixptr->AddSource(src6);
    test->mixptr->RemoveSource(src1);

    while (iR1 && iR2 && iR3 && iR4 && iR5 && iR6)
    {
        test->mixptr->Mix(iChannels, &audioframe);

        fwrite(audioframe.data_, sizeof(int16_t), iR1, pOut);

        iR1 = fread(pBuf1, sizeof(short), i10msSz, pSrc1);
        iR2 = fread(pBuf2, sizeof(short), i10msSz, pSrc2);
        iR3 = fread(pBuf3, sizeof(short), i10msSz, pSrc3);
        iR4 = fread(pBuf4, sizeof(short), i10msSz, pSrc4);
        iR5 = fread(pBuf5, sizeof(short), i10msSz, pSrc5);
        iR6 = fread(pBuf6, sizeof(short), i10msSz, pSrc6);

        src1->fillaudioframe(pBuf1, iR1);
        src2->fillaudioframe(pBuf2, iR2);
        src3->fillaudioframe(pBuf3, iR3);
        // src4->fillaudioframe(pBuf4, iR4);
        // src5->fillaudioframe(pBuf5, iR5);
        // src6->fillaudioframe(pBuf6, iR6);
    }

    delete test; // cannot use free(test); it won't call ~xxx functions
    // delete mixptr; // cannot do this, scoped_refptr is auto deleted

    fclose(pSrc1);
    fclose(pSrc2);
    fclose(pSrc3);
    fclose(pSrc4);
    fclose(pSrc5);
    fclose(pSrc6);
    fclose(pOut);

    return 0;
}