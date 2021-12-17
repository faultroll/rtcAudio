/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_AUDIO_CONFERENCE_MIXER_SOURCE_AUDIO_CONFERENCE_MIXER_IMPL_H_
#define WEBRTC_MODULES_AUDIO_CONFERENCE_MIXER_SOURCE_AUDIO_CONFERENCE_MIXER_IMPL_H_

#include <stdint.h>

#include <list>
#include <vector>
#include <map>
#include <memory>

#include "rtc_base/critical_section.h"
#include "modules/audio_conference_mixer/include/audio_conference_mixer.h"
#include "modules/audio_conference_mixer/source/memory_pool.h"
#include "modules/audio_conference_mixer/source/time_scheduler.h"
#include "modules/include/audio_frame.h"
// #include "modules/audio_processing/include/gain_control.h"
// #include "modules/audio_processing/include/voice_detection.h"

namespace webrtc {
class AudioBuffer;
/* class ApmDataDumper;
// class FixedGainController;
class Limiter; */
class GainControl;
class StandaloneVad;

struct FrameAndMuteInfo {
  FrameAndMuteInfo(AudioFrame* f, bool m) : frame(f), muted(m) {}
  AudioFrame* frame;
  bool muted;
};

typedef std::list<FrameAndMuteInfo> AudioFrameList;
typedef std::list<MixerParticipant*> MixerParticipantList;

// Cheshire cat implementation of MixerParticipant's non virtual functions.
class MixHistory
{
public:
    MixHistory();
    ~MixHistory();

    // Returns true if the participant is being mixed.
    bool IsMixed() const;

    // Returns true if the participant was mixed previous mix
    // iteration.
    bool WasMixed() const;

    // Updates the mixed status.
    int32_t SetIsMixed(bool mixed);

    void ResetMixedStatus();
private:
    bool _isMixed;
};

class AudioConferenceMixerImpl : public AudioConferenceMixer
{
public:
    // AudioProcessing only accepts 10 ms frames.
    enum {kProcessPeriodicityInMs = 10};

    AudioConferenceMixerImpl(int id);
    ~AudioConferenceMixerImpl();

    // Must be called after ctor.
    bool Init();

    // Module functions
    int64_t TimeUntilNextProcess() override;
    void Process() override;

    // AudioConferenceMixer functions
    int32_t RegisterMixedStreamCallback(
        AudioMixerOutputReceiver* mixReceiver) override;
    int32_t UnRegisterMixedStreamCallback() override;
    int32_t SetMixabilityStatus(MixerParticipant* participant,
                                bool mixable) override;
    bool MixabilityStatus(const MixerParticipant& participant) const override;
    int32_t SetMinimumMixingFrequency(Frequency freq) override;
    int32_t SetAnonymousMixabilityStatus(
        MixerParticipant* participant, bool mixable) override;
    bool AnonymousMixabilityStatus(
        const MixerParticipant& participant) const override;

    // woogeen vad
    int32_t RegisterMixerVadCallback(AudioMixerVadReceiver *vadReceiver,
                                        const uint32_t amountOf10MsBetweenCallbacks) override;
    int32_t UnRegisterMixerVadCallback() override;

    void SetMultipleInputs(bool enable) override;

private:
    enum{DEFAULT_AUDIO_FRAME_POOLSIZE = 50};

    // Set/get mix frequency
    int32_t SetOutputFrequency(const Frequency& frequency);
    Frequency OutputFrequency() const;

    // Fills mixList with the AudioFrames pointers that should be used when
    // mixing.
    // maxAudioFrameCounter both input and output specifies how many more
    // AudioFrames that are allowed to be mixed.
    // rampOutList contain AudioFrames corresponding to an audio stream that
    // used to be mixed but shouldn't be mixed any longer. These AudioFrames
    // should be ramped out over this AudioFrame to avoid audio discontinuities.
    void UpdateToMix(
        AudioFrameList* mixList,
        AudioFrameList* rampOutList,
        std::map<int, MixerParticipant*>* mixParticipantList,
        size_t* maxAudioFrameCounter) const;

    // Return the lowest mixing frequency that can be used without having to
    // downsample any audio.
    int32_t GetLowestMixingFrequency() const;
    int32_t GetLowestMixingFrequencyFromList(
        const MixerParticipantList& mixList) const;

    // Return the AudioFrames that should be mixed anonymously.
    void GetAdditionalAudio(AudioFrameList* additionalFramesList) const;

    // Update the MixHistory of all MixerParticipants. mixedParticipantsList
    // should contain a map of MixerParticipants that have been mixed.
    void UpdateMixedStatus(
        const std::map<int, MixerParticipant*>& mixedParticipantsList) const;

    // Clears audioFrameList and reclaims all memory associated with it.
    void ClearAudioFrameList(AudioFrameList* audioFrameList) const;

    // This function returns true if it finds the MixerParticipant in the
    // specified list of MixerParticipants.
    bool IsParticipantInList(const MixerParticipant& participant,
                             const MixerParticipantList& participantList) const;

    // Add/remove the MixerParticipant to the specified
    // MixerParticipant list.
    bool AddParticipantToList(
        MixerParticipant* participant,
        MixerParticipantList* participantList) const;
    bool RemoveParticipantFromList(
        MixerParticipant* removeParticipant,
        MixerParticipantList* participantList) const;

    // Mix the AudioFrames stored in audioFrameList into mixedAudio.
    int32_t MixFromList(AudioFrame* mixedAudio,
                        const AudioFrameList& audioFrameList) const;

    // Mix the AudioFrames stored in audioFrameList into mixedAudio. No
    // record will be kept of this mix (e.g. the corresponding MixerParticipants
    // will not be marked as IsMixed()
    int32_t MixAnonomouslyFromList(AudioFrame* mixedAudio,
                                   const AudioFrameList& audioFrameList) const;

    bool LimitMixedAudio(AudioFrame* mixedAudio) const;

    rtc::CriticalSection _crit;
    rtc::CriticalSection _cbCrit;

    int32_t _id;

    Frequency _minimumMixingFreq;

    // Mix result callback
    AudioMixerOutputReceiver* _mixReceiver;

    // The current sample frequency and sample size when mixing.
    Frequency _outputFrequency;
    size_t _sampleSize;

    // Memory pool to avoid allocating/deallocating AudioFrames
    MemoryPool<AudioFrame>* _audioFramePool;

    // List of all participants. Note all lists are disjunct
    MixerParticipantList _participantList;              // May be mixed.
    // Always mixed, anonomously.
    MixerParticipantList _additionalParticipantList;

    size_t _numMixedParticipants;
    // Determines if we will use a limiter for clipping protection during
    // mixing.
    bool _use_limiter;

    uint32_t _timeStamp;

    // Metronome class.
    TimeScheduler _timeScheduler;

    // Counter keeping track of concurrent calls to process.
    // Note: should never be higher than 1 or lower than 0.
    int16_t _processCalls;

    // Used for inhibiting saturation in mixing.
    rtc::CriticalSection _crit_render RTC_ACQUIRED_BEFORE(_crit_capture);
    rtc::CriticalSection _crit_capture;
    std::unique_ptr<GainControl> _limiter;
    /* std::unique_ptr<ApmDataDumper> _data_dumper;
    // std::unique_ptr<FixedGainController> _limiter;
    std::unique_ptr<Limiter> _limiter; */
    std::unique_ptr<AudioBuffer> _mixed_buffer;

    // woogeen vad
    enum {kMaximumVadParticipants = 1024};

    // Combine energy with vad probability
    uint32_t CombinedEnergy(AudioFrame* frame) const;
    void UpdateVadStatistics(AudioFrameList* mixList);

    bool _vadEnabled;
    AudioMixerVadReceiver* _vadReceiver;
    uint32_t _amountOf10MsBetweenVadCallbacks;

    uint32_t _amountOf10MsAll;
    uint32_t _amountOf10MsRemainder;
    std::map<int32_t, int64_t> _vadParticipantEnergyList;

    // std::map<int32_t, std::unique_ptr<VoiceDetection>> _vads;
    // std::map<int32_t, std::unique_ptr<rtc::CriticalSection>> _vad_crits;
    // std::map<int32_t, std::unique_ptr<AudioBuffer>> _vad_buffers;
    mutable std::map<int32_t, std::unique_ptr<StandaloneVad>> _vads;

    std::vector<ParticipantVadStatistics> _vadStatistics;

    bool _supportMultipleInputs;
};
}  // namespace webrtc

#endif // WEBRTC_MODULES_AUDIO_CONFERENCE_MIXER_SOURCE_AUDIO_CONFERENCE_MIXER_IMPL_H_
