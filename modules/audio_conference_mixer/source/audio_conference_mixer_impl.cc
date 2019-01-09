/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_conference_mixer/source/audio_conference_mixer_impl.h"

#include <stddef.h>
#include <stdint.h>

#include "modules/include/audio_frame_operations.h"
#include "modules/audio_conference_mixer/include/audio_conference_mixer_defines.h"
#include "modules/audio_conference_mixer/source/audio_frame_manipulator.h"
#include "modules/audio_processing/audio_buffer.h"
#include "modules/audio_processing/include/common.h"
#include "modules/audio_processing/agc/gain_control_impl.h"
/* #include "modules/audio_processing/high_pass_filter.h" */
/* // #include "modules/audio_processing/transient/transient_suppressor.h"
#include "modules/audio_processing/ns/noise_suppression_impl.h" */
/* #include "modules/audio_processing/logging/apm_data_dumper.h"
// #include "modules/audio_processing/agc2/fixed_gain_controller.h"
#include "modules/audio_processing/agc2/limiter.h" */
// #include "modules/audio_processing/vad/voice_detection_impl.h"
#include "modules/audio_processing/vad/standalone_vad.h"
#include "rtc_base/trace.h"

// #define _VAD_METHOD_ENERGY_
// #define _VAD_METHOD_VOICE_DETECTION_
#define _VAD_METHOD_JOINT_ENERGY_VOICE_DETECTION_

#define GROUP_ID(id) (((id) >> 16) & 0xffff)

namespace webrtc {
namespace {

struct ParticipantFrameStruct {
  ParticipantFrameStruct(MixerParticipant* p, AudioFrame* a, bool m)
      : participant(p), audioFrame(a), muted(m) {}
  MixerParticipant* participant;
  AudioFrame* audioFrame;
  bool muted;
};

typedef std::list<ParticipantFrameStruct*> ParticipantFrameStructList;

// Mix |frame| into |mixed_frame|, with saturation protection and upmixing.
// These effects are applied to |frame| itself prior to mixing. Assumes that
// |mixed_frame| always has at least as many channels as |frame|. Supports
// stereo at most.
//
// TODO(andrew): consider not modifying |frame| here.
void MixFrames(AudioFrame* mixed_frame, AudioFrame* frame, bool use_limiter) {
  assert(mixed_frame->num_channels_ >= frame->num_channels_);
  if (use_limiter) {
    // This is to avoid saturation in the mixing. It is only
    // meaningful if the limiter will be used.
    AudioFrameOperations::ApplyHalfGain(frame);
  }
  if (mixed_frame->num_channels_ > frame->num_channels_) {
    // We only support mono-to-stereo.
    assert(mixed_frame->num_channels_ == 2 &&
           frame->num_channels_ == 1);
    AudioFrameOperations::MonoToStereo(frame);
  }

  AudioFrameOperations::Add(*frame, mixed_frame);
}

// Return the max number of channels from a |list| composed of AudioFrames.
size_t MaxNumChannels(const AudioFrameList* list) {
  size_t max_num_channels = 1;
  for (AudioFrameList::const_iterator iter = list->begin();
       iter != list->end();
       ++iter) {
    max_num_channels = std::max(max_num_channels, iter->frame->num_channels_);
  }
  return max_num_channels;
}

}  // namespace

MixerParticipant::MixerParticipant()
    : _mixHistory(new MixHistory()) {
}

MixerParticipant::~MixerParticipant() {
    delete _mixHistory;
}

bool MixerParticipant::IsMixed() const {
    return _mixHistory->IsMixed();
}

MixHistory::MixHistory()
    : _isMixed(0) {
}

MixHistory::~MixHistory() {
}

bool MixHistory::IsMixed() const {
    return _isMixed;
}

bool MixHistory::WasMixed() const {
    // Was mixed is the same as is mixed depending on perspective. This function
    // is for the perspective of AudioConferenceMixerImpl.
    return IsMixed();
}

int32_t MixHistory::SetIsMixed(const bool mixed) {
    _isMixed = mixed;
    return 0;
}

void MixHistory::ResetMixedStatus() {
    _isMixed = false;
}

AudioConferenceMixer* AudioConferenceMixer::Create(int id) {
    AudioConferenceMixerImpl* mixer = new AudioConferenceMixerImpl(id);
    if(!mixer->Init()) {
        delete mixer;
        return NULL;
    }
    return mixer;
}

AudioConferenceMixerImpl::AudioConferenceMixerImpl(int id)
    : _id(id),
      _minimumMixingFreq(kLowestPossible),
      _mixReceiver(NULL),
      _outputFrequency(kDefaultFrequency),
      _sampleSize(0),
      _audioFramePool(NULL),
      _participantList(),
      _additionalParticipantList(),
      _numMixedParticipants(0),
      _use_limiter(true),
      _timeStamp(0),
      _timeScheduler(kProcessPeriodicityInMs),
      _processCalls(0),
      _vadEnabled(false),
      _vadReceiver(NULL),
      _amountOf10MsBetweenVadCallbacks(0),
      _amountOf10MsAll(0),
      _amountOf10MsRemainder(0),
      _supportMultipleInputs(false) {}

bool AudioConferenceMixerImpl::Init() {
    _limiter.reset(
        new GainControlImpl(&_crit_render, &_crit_capture));
    /* _data_dumper = std::unique_ptr<ApmDataDumper>(new ApmDataDumper(0));
    // histogram_name_prefix can only be |AudioMixer|/|Agc2|/|Test|
    // _limiter.reset(
    //     new FixedGainController(_data_dumper.get(), "AudioMixer");
    _limiter.reset(
        new Limiter(static_cast<size_t>(48000), _data_dumper.get(), "AudioMixer"));
    // _limiter = rtc::make_unique<Limiter>(
    //     static_cast<size_t>(48000), _data_dumper.get(), "AudioMixer"); // only one brace! */
    if(!_limiter.get())
        return false;

    MemoryPool<AudioFrame>::CreateMemoryPool(_audioFramePool,
                                             DEFAULT_AUDIO_FRAME_POOLSIZE);
    if(_audioFramePool == NULL)
        return false;

    if(SetOutputFrequency(kDefaultFrequency) == -1)
        return false;

    size_t num_channels = 1;
    _limiter->Initialize(num_channels, _outputFrequency);
    if(_limiter->set_mode(GainControl::kFixedDigital) != AudioProcessing::kNoError)
        return false;
    // We smoothly limit the mixed frame to -7 dbFS. -6 would correspond to the
    // divide-by-2 but -7 is used instead to give a bit of headroom since the
    // AGC is not a hard limiter.
    if(_limiter->set_target_level_dbfs(7) != AudioProcessing::kNoError)
        return false;
    if(_limiter->set_compression_gain_db(0) != AudioProcessing::kNoError)
        return false;
    if(_limiter->enable_limiter(true) != AudioProcessing::kNoError)
        return false;
    if(_limiter->Enable(true) != AudioProcessing::kNoError)
        return false;
    /* // _limiter.SetGain(0.f); */
    /* _filter.reset(
        new HighPassFilter(_outputFrequency, num_channels)); */
    /* // _suppressor.reset(
    //     new TransientSuppressor());
    // _suppressor->Initialize(_outputFrequency, _outputFrequency, num_channels);
    _suppressor.reset(
        new NoiseSuppressionImpl(&_crit_capture));
    _suppressor->Initialize(num_channels, _outputFrequency);
    // _suppressor->set_level(NoiseSuppression::kModerate);
    _suppressor->Enable(true); */

    _mixed_buffer.reset(
        new AudioBuffer(_outputFrequency, num_channels,
                        _outputFrequency, num_channels,
                        _outputFrequency, num_channels));

    return true;
}

AudioConferenceMixerImpl::~AudioConferenceMixerImpl() {
    MemoryPool<AudioFrame>::DeleteMemoryPool(_audioFramePool);
    assert(_audioFramePool == NULL);
}

// Process should be called every kProcessPeriodicityInMs ms
int64_t AudioConferenceMixerImpl::TimeUntilNextProcess() {
    int64_t timeUntilNextProcess = 0;
    rtc::CritScope cs(&_crit);
    if(_timeScheduler.TimeToNextUpdate(timeUntilNextProcess) != 0) {
        WEBRTC_TRACE(kTraceError, kTraceAudioMixerServer, _id,
                     "failed in TimeToNextUpdate() call");
        // Sanity check
        assert(false);
        return -1;
    }
    return timeUntilNextProcess;
}

void AudioConferenceMixerImpl::Process() {
    size_t remainingParticipantsAllowedToMix =
        kMaximumAmountOfMixedParticipants;
    {
        rtc::CritScope cs(&_crit);
        assert(_processCalls == 0);
        _processCalls++;

        // Let the scheduler know that we are running one iteration.
        _timeScheduler.UpdateScheduler();
    }

    AudioFrameList mixList;
    AudioFrameList rampOutList;
    AudioFrameList additionalFramesList;
    std::map<int, MixerParticipant*> mixedParticipantsMap;
    {
        rtc::CritScope cs(&_cbCrit);

        int32_t lowFreq = GetLowestMixingFrequency();
        // SILK can run in 12 kHz and 24 kHz. These frequencies are not
        // supported so use the closest higher frequency to not lose any
        // information.
        // TODO(henrike): this is probably more appropriate to do in
        //                GetLowestMixingFrequency().
        if (lowFreq == 12000) {
            lowFreq = 16000;
        } else if (lowFreq == 24000) {
            lowFreq = 32000;
        }
        if(lowFreq <= 0) {
          rtc::CritScope cs(&_crit);
          _processCalls--;
          return;
        } else {
            switch(lowFreq) {
            case 8000:
                if(OutputFrequency() != kNbInHz) {
                    SetOutputFrequency(kNbInHz);
                }
                break;
            case 16000:
                if(OutputFrequency() != kWbInHz) {
                    SetOutputFrequency(kWbInHz);
                }
                break;
            case 32000:
                if(OutputFrequency() != kSwbInHz) {
                    SetOutputFrequency(kSwbInHz);
                }
                break;
            case 48000:
                if(OutputFrequency() != kFbInHz) {
                    SetOutputFrequency(kFbInHz);
                }
                break;
            default:
                assert(false);

                rtc::CritScope cs(&_crit);
                _processCalls--;
                return;
            }
        }

        UpdateToMix(&mixList, &rampOutList, &mixedParticipantsMap,
                    &remainingParticipantsAllowedToMix);

        GetAdditionalAudio(&additionalFramesList);
        UpdateMixedStatus(mixedParticipantsMap);
    }

    AudioFrame* generalFrame = NULL;
    if(_audioFramePool->PopMemory(generalFrame) == -1) {
        WEBRTC_TRACE(kTraceMemory, kTraceAudioMixerServer, _id,
                     "failed PopMemory() call");
        assert(false);
        return;
    }

    AudioFrame* uniqueFrames[kMaximumAmountOfMixedParticipants];
    for (size_t i = 0; i < mixList.size(); ++i) {
        if(_audioFramePool->PopMemory(uniqueFrames[i]) == -1) {
            WEBRTC_TRACE(kTraceMemory, kTraceAudioMixerServer, _id,
                    "failed PopMemory() call");
            assert(false);
            return;
        }
    }

    uint32_t mixedFrameCount = 0;
    bool fireVadCallback = false;
    {
        rtc::CritScope cs(&_crit);

        // woogeen vad
        if(_vadEnabled) {
            if (_amountOf10MsRemainder == 0) {
                _amountOf10MsAll = _amountOf10MsBetweenVadCallbacks;
                _amountOf10MsRemainder = _amountOf10MsBetweenVadCallbacks;
                _vadParticipantEnergyList.clear();
            }

            if (mixList.size() + additionalFramesList.size() <= kMaximumVadParticipants) {
                AudioFrameList vadParticipantList;

                if (mixList.size() > 0)
                    vadParticipantList.insert(vadParticipantList.end(), mixList.begin(), mixList.end());

                if (additionalFramesList.size() > 0)
                    vadParticipantList.insert(vadParticipantList.end(), additionalFramesList.begin(), additionalFramesList.end());

                UpdateVadStatistics(&vadParticipantList);
            } else {
                WEBRTC_TRACE(kTraceError, kTraceAudioMixerServer, _id,
                        "vad participants exceeded MaximumAmount(%d)", kMaximumVadParticipants);
            }
        }

        // TODO(henrike): it might be better to decide the number of channels
        //                with an API instead of dynamically.

        // Find the max channels over all mixing lists.
        size_t num_mixed_channels;

        // force dual channels for 48000hz output
        if (_outputFrequency < 48000)
            num_mixed_channels = std::max(MaxNumChannels(&mixList),
            std::max(MaxNumChannels(&additionalFramesList),
                     MaxNumChannels(&rampOutList)));
        else
            num_mixed_channels = 2;

        // We only use the limiter if it supports the output sample rate and
        // we're actually mixing multiple streams.
        _use_limiter = false && // disable limiter
            _numMixedParticipants > 1 &&
            _outputFrequency <= AudioProcessing::kMaxNativeSampleRateHz;

        // 1. mix AnonomouslyFromList
        generalFrame->UpdateFrame(0, NULL, 0, _outputFrequency,
                AudioFrame::kNormalSpeech,
                AudioFrame::kVadPassive, num_mixed_channels, -1);

        MixAnonomouslyFromList(generalFrame, additionalFramesList);
        MixAnonomouslyFromList(generalFrame, rampOutList);

        // 2. mix workList and anonomouslyMixerdFrame
        std::map<uint16_t, bool> groupIds;
        for (size_t i = 0; i <= mixList.size(); ++i) {
            AudioFrameList workList = mixList;
            int id = -1;
            AudioFrame* mixedAudio = NULL;

            if (i == mixList.size()) {
                mixedAudio = generalFrame;
            } else {
                AudioFrameList::iterator it = workList.begin();
                advance(it, i);
                id = (*it).frame->id_;

                if (_supportMultipleInputs) {
                    uint16_t groupId = GROUP_ID(id);
                    if (groupIds.find(groupId) != groupIds.end()) {
                        continue;
                    }

                    groupIds[groupId] = true;
                    for (it = workList.begin(); it != workList.end(); ) {
                        if (GROUP_ID((*it).frame->id_) == groupId) {
                            it = workList.erase(it);
                        } else {
                            ++it;
                        }
                    }
                } else {
                    workList.erase(it);
                }

                mixedAudio = uniqueFrames[mixedFrameCount++];
                mixedAudio->CopyFrom(*generalFrame);
            }

            MixFromList(mixedAudio, workList);

            if(mixedAudio->samples_per_channel_ == 0) {
                // Nothing was mixed, set the audio samples to silence.
                mixedAudio->samples_per_channel_ = _sampleSize;
                AudioFrameOperations::Mute(mixedAudio);
            } else {
                // Only call the limiter if we have something to mix.
                LimitMixedAudio(mixedAudio);
            }

            mixedAudio->id_ = id;
            mixedAudio->timestamp_ = _timeStamp * mixedAudio->sample_rate_hz_ / 1000;
        }

        _timeStamp += 10;

        // woogeen vad
        if(_vadEnabled && --_amountOf10MsRemainder == 0) {
            _vadStatistics.resize(_vadParticipantEnergyList.size());

            size_t i = 0;
            for (auto& item : _vadParticipantEnergyList) {
                _vadStatistics[i].id        = item.first;
                _vadStatistics[i].energy    = item.second / _amountOf10MsAll;

                i++;
            }
            fireVadCallback = true;
        }
    }

    {
        rtc::CritScope cs(&_cbCrit);
        if(_mixReceiver != NULL) {
            _mixReceiver->NewMixedAudio(
                _id,
                *generalFrame,
                const_cast<const AudioFrame**>(uniqueFrames),
                mixedFrameCount);
        }

        if(_vadReceiver != NULL && fireVadCallback) {
            _vadReceiver->VadParticipants(
                    _vadStatistics.data(),
                    _vadStatistics.size());
        }
    }

    // Reclaim all outstanding memory.
    _audioFramePool->PushMemory(generalFrame);
    for (size_t i = 0; i < mixList.size(); ++i) {
        _audioFramePool->PushMemory(uniqueFrames[i]);
    }
    ClearAudioFrameList(&mixList);
    ClearAudioFrameList(&rampOutList);
    ClearAudioFrameList(&additionalFramesList);
    {
        rtc::CritScope cs(&_crit);
        _processCalls--;
    }
    return;
}

int32_t AudioConferenceMixerImpl::RegisterMixedStreamCallback(
    AudioMixerOutputReceiver* mixReceiver) {
    rtc::CritScope cs(&_cbCrit);
    if(_mixReceiver != NULL) {
        return -1;
    }
    _mixReceiver = mixReceiver;
    return 0;
}

int32_t AudioConferenceMixerImpl::UnRegisterMixedStreamCallback() {
    rtc::CritScope cs(&_cbCrit);
    if(_mixReceiver == NULL) {
        return -1;
    }
    _mixReceiver = NULL;
    return 0;
}

int32_t AudioConferenceMixerImpl::SetOutputFrequency(
    const Frequency& frequency) {
    rtc::CritScope cs(&_crit);

    _outputFrequency = frequency;
    _sampleSize =
        static_cast<size_t>((_outputFrequency*kProcessPeriodicityInMs) / 1000);

    return 0;
}

AudioConferenceMixer::Frequency
AudioConferenceMixerImpl::OutputFrequency() const {
    rtc::CritScope cs(&_crit);
    return _outputFrequency;
}

int32_t AudioConferenceMixerImpl::SetMixabilityStatus(
    MixerParticipant* participant, bool mixable) {
    if (!mixable) {
        // Anonymous participants are in a separate list. Make sure that the
        // participant is in the _participantList if it is being mixed.
        SetAnonymousMixabilityStatus(participant, false);
    }
    size_t numMixedParticipants;
    {
        rtc::CritScope cs(&_cbCrit);
        const bool isMixed =
            IsParticipantInList(*participant, _participantList);
        // API must be called with a new state.
        if(!(mixable ^ isMixed)) {
            WEBRTC_TRACE(kTraceWarning, kTraceAudioMixerServer, _id,
                         "Mixable is already %s",
                         isMixed ? "ON" : "OFF");
            return -1;
        }
        bool success = false;
        if(mixable) {
            success = AddParticipantToList(participant, &_participantList);
        } else {
            success = RemoveParticipantFromList(participant, &_participantList);
        }
        if(!success) {
            WEBRTC_TRACE(kTraceError, kTraceAudioMixerServer, _id,
                         "failed to %s participant",
                         mixable ? "add" : "remove");
            assert(false);
            return -1;
        }

        size_t numMixedNonAnonymous = _participantList.size();
        if (numMixedNonAnonymous > kMaximumAmountOfMixedParticipants) {
            numMixedNonAnonymous = kMaximumAmountOfMixedParticipants;
        }
        numMixedParticipants =
            numMixedNonAnonymous + _additionalParticipantList.size();
    }
    // A MixerParticipant was added or removed. Make sure the scratch
    // buffer is updated if necessary.
    // Note: The scratch buffer may only be updated in Process().
    rtc::CritScope cs(&_crit);
    _numMixedParticipants = numMixedParticipants;

    /* if (!mixable) ClearVads(); */ // TODO(lgY): should we clear all vads here?

    return 0;
}

bool AudioConferenceMixerImpl::MixabilityStatus(
    const MixerParticipant& participant) const {
    rtc::CritScope cs(&_cbCrit);
    return IsParticipantInList(participant, _participantList);
}

int32_t AudioConferenceMixerImpl::SetAnonymousMixabilityStatus(
    MixerParticipant* participant, bool anonymous) {
    rtc::CritScope cs(&_cbCrit);
    if(IsParticipantInList(*participant, _additionalParticipantList)) {
        if(anonymous) {
            return 0;
        }
        if(!RemoveParticipantFromList(participant,
                                      &_additionalParticipantList)) {
            WEBRTC_TRACE(kTraceError, kTraceAudioMixerServer, _id,
                         "unable to remove participant from anonymous list");
            assert(false);
            return -1;
        }
        return AddParticipantToList(participant, &_participantList) ? 0 : -1;
    }
    if(!anonymous) {
        return 0;
    }
    const bool mixable = RemoveParticipantFromList(participant,
                                                   &_participantList);
    if(!mixable) {
        WEBRTC_TRACE(
            kTraceWarning,
            kTraceAudioMixerServer,
            _id,
            "participant must be registered before turning it into anonymous");
        // Setting anonymous status is only possible if MixerParticipant is
        // already registered.
        return -1;
    }
    return AddParticipantToList(participant, &_additionalParticipantList) ?
        0 : -1;
}

bool AudioConferenceMixerImpl::AnonymousMixabilityStatus(
    const MixerParticipant& participant) const {
    rtc::CritScope cs(&_cbCrit);
    return IsParticipantInList(participant, _additionalParticipantList);
}

int32_t AudioConferenceMixerImpl::SetMinimumMixingFrequency(
    Frequency freq) {
    // Make sure that only allowed sampling frequencies are used. Use closest
    // higher sampling frequency to avoid losing information.
    if (static_cast<int>(freq) == 12000) {
         freq = kWbInHz;
    } else if (static_cast<int>(freq) == 24000) {
        freq = kSwbInHz;
    }

    if((freq == kNbInHz) || (freq == kWbInHz) || (freq == kSwbInHz) || (freq == kFbInHz ) ||
       (freq == kLowestPossible)) {
        _minimumMixingFreq=freq;
        return 0;
    } else {
        WEBRTC_TRACE(kTraceError, kTraceAudioMixerServer, _id,
                     "SetMinimumMixingFrequency incorrect frequency: %i",freq);
        assert(false);
        return -1;
    }
}

// Check all AudioFrames that are to be mixed. The highest sampling frequency
// found is the lowest that can be used without losing information.
int32_t AudioConferenceMixerImpl::GetLowestMixingFrequency() const {
    const int participantListFrequency =
        GetLowestMixingFrequencyFromList(_participantList);
    const int anonymousListFrequency =
        GetLowestMixingFrequencyFromList(_additionalParticipantList);
    const int highestFreq =
        (participantListFrequency > anonymousListFrequency) ?
            participantListFrequency : anonymousListFrequency;
    // Check if the user specified a lowest mixing frequency.
    if(_minimumMixingFreq != kLowestPossible) {
        if(_minimumMixingFreq > highestFreq) {
            return _minimumMixingFreq;
        }
    }
    return highestFreq;
}

int32_t AudioConferenceMixerImpl::GetLowestMixingFrequencyFromList(
    const MixerParticipantList& mixList) const {
    int32_t highestFreq = 8000;
    for (MixerParticipantList::const_iterator iter = mixList.begin();
         iter != mixList.end();
         ++iter) {
        const int32_t neededFrequency = (*iter)->NeededFrequency(_id);
        if(neededFrequency > highestFreq) {
            highestFreq = neededFrequency;
        }
    }
    return highestFreq;
}

void AudioConferenceMixerImpl::UpdateToMix(
    AudioFrameList* mixList,
    AudioFrameList* rampOutList,
    std::map<int, MixerParticipant*>* mixParticipantList,
    size_t* maxAudioFrameCounter) const {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                 "UpdateToMix(mixList,rampOutList,mixParticipantList,%d)",
                 *maxAudioFrameCounter);
    const size_t mixListStartSize = mixList->size();
    AudioFrameList activeList;
    // Struct needed by the passive lists to keep track of which AudioFrame
    // belongs to which MixerParticipant.
    ParticipantFrameStructList passiveWasNotMixedList;
    ParticipantFrameStructList passiveWasMixedList;
    for (MixerParticipantList::const_iterator participant =
        _participantList.begin(); participant != _participantList.end();
         ++participant) {
        // Stop keeping track of passive participants if there are already
        // enough participants available (they wont be mixed anyway).
        bool mustAddToPassiveList = (*maxAudioFrameCounter >
                                    (activeList.size() +
                                     passiveWasMixedList.size() +
                                     passiveWasNotMixedList.size()));

        bool wasMixed = false;
        wasMixed = (*participant)->_mixHistory->WasMixed();
        AudioFrame* audioFrame = NULL;
        if(_audioFramePool->PopMemory(audioFrame) == -1) {
            WEBRTC_TRACE(kTraceMemory, kTraceAudioMixerServer, _id,
                         "failed PopMemory() call");
            assert(false);
            return;
        }
        audioFrame->sample_rate_hz_ = _outputFrequency;

        auto ret = (*participant)->GetAudioFrameWithMuted(_id, audioFrame);
        if (ret == MixerParticipant::AudioFrameInfo::kError) {
            WEBRTC_TRACE(kTraceWarning, kTraceAudioMixerServer, _id,
                         "failed to GetAudioFrameWithMuted() from participant");
            _audioFramePool->PushMemory(audioFrame);
            continue;
        }
        const bool muted = (ret == MixerParticipant::AudioFrameInfo::kMuted);
        if (_participantList.size() != 1) {
          // TODO(wu): Issue 3390, add support for multiple participants case.
          audioFrame->ntp_time_ms_ = -1;
        }

        // TODO(henrike): this assert triggers in some test cases where SRTP is
        // used which prevents NetEQ from making a VAD. Temporarily disable this
        // assert until the problem is fixed on a higher level.
        // assert(audioFrame->vad_activity_ != AudioFrame::kVadUnknown);
        if (audioFrame->vad_activity_ == AudioFrame::kVadUnknown) {
            WEBRTC_TRACE(kTraceWarning, kTraceAudioMixerServer, _id,
                         "invalid VAD state from participant");
        }

        if(audioFrame->vad_activity_ == AudioFrame::kVadActive) {
            if(!wasMixed && !muted) {
                RampIn(*audioFrame);
            }

            if(activeList.size() >= *maxAudioFrameCounter) {
                // There are already more active participants than should be
                // mixed. Only keep the ones with the highest energy.
                AudioFrameList::iterator replaceItem;
                uint32_t lowestEnergy =
                    muted ? 0 : CombinedEnergy(audioFrame);

                bool found_replace_item = false;
                for (AudioFrameList::iterator iter = activeList.begin();
                     iter != activeList.end();
                     ++iter) {
                    const uint32_t energy =
                        muted ? 0 : CombinedEnergy(iter->frame);
                    if(energy < lowestEnergy) {
                        replaceItem = iter;
                        lowestEnergy = energy;
                        found_replace_item = true;
                    }
                }
                if(found_replace_item) {
                    RTC_DCHECK(!muted);  // Cannot replace with a muted frame.
                    FrameAndMuteInfo replaceFrame = *replaceItem;

                    bool replaceWasMixed = false;
                    std::map<int, MixerParticipant*>::const_iterator it =
                        mixParticipantList->find(replaceFrame.frame->id_);

                    // When a frame is pushed to |activeList| it is also pushed
                    // to mixParticipantList with the frame's id. This means
                    // that the Find call above should never fail.
                    assert(it != mixParticipantList->end());
                    replaceWasMixed = it->second->_mixHistory->WasMixed();

                    mixParticipantList->erase(replaceFrame.frame->id_);
                    activeList.erase(replaceItem);

                    activeList.push_front(FrameAndMuteInfo(audioFrame, muted));
                    (*mixParticipantList)[audioFrame->id_] = *participant;
                    assert(mixParticipantList->size() <=
                           kMaximumAmountOfMixedParticipants);

                    if (replaceWasMixed) {
                      if (!replaceFrame.muted) {
                        RampOut(*replaceFrame.frame);
                      }
                      rampOutList->push_back(replaceFrame);
                      assert(rampOutList->size() <=
                             kMaximumAmountOfMixedParticipants);
                    } else {
                      _audioFramePool->PushMemory(replaceFrame.frame);
                    }
                } else {
                    if(wasMixed) {
                        if (!muted) {
                            RampOut(*audioFrame);
                        }
                        rampOutList->push_back(FrameAndMuteInfo(audioFrame,
                                                                muted));
                        assert(rampOutList->size() <=
                               kMaximumAmountOfMixedParticipants);
                    } else {
                        _audioFramePool->PushMemory(audioFrame);
                    }
                }
            } else {
                activeList.push_front(FrameAndMuteInfo(audioFrame, muted));
                (*mixParticipantList)[audioFrame->id_] = *participant;
                assert(mixParticipantList->size() <=
                       kMaximumAmountOfMixedParticipants);
            }
        } else {
            if(wasMixed) {
                ParticipantFrameStruct* part_struct =
                    new ParticipantFrameStruct(*participant, audioFrame, muted);
                passiveWasMixedList.push_back(part_struct);
            } else if(mustAddToPassiveList) {
                if (!muted) {
                    RampIn(*audioFrame);
                }
                ParticipantFrameStruct* part_struct =
                    new ParticipantFrameStruct(*participant, audioFrame, muted);
                passiveWasNotMixedList.push_back(part_struct);
            } else {
                _audioFramePool->PushMemory(audioFrame);
            }
        }
    }
    assert(activeList.size() <= *maxAudioFrameCounter);
    // At this point it is known which participants should be mixed. Transfer
    // this information to this functions output parameters.
    for (AudioFrameList::const_iterator iter = activeList.begin();
         iter != activeList.end();
         ++iter) {
        mixList->push_back(*iter);
    }
    activeList.clear();
    // Always mix a constant number of AudioFrames. If there aren't enough
    // active participants mix passive ones. Starting with those that was mixed
    // last iteration.
    for (ParticipantFrameStructList::const_iterator
        iter = passiveWasMixedList.begin(); iter != passiveWasMixedList.end();
         ++iter) {
        if(mixList->size() < *maxAudioFrameCounter + mixListStartSize) {
            mixList->push_back(FrameAndMuteInfo((*iter)->audioFrame,
                                                (*iter)->muted));
            (*mixParticipantList)[(*iter)->audioFrame->id_] =
                (*iter)->participant;
            assert(mixParticipantList->size() <=
                   kMaximumAmountOfMixedParticipants);
        } else {
            _audioFramePool->PushMemory((*iter)->audioFrame);
        }
        delete *iter;
    }
    // And finally the ones that have not been mixed for a while.
    for (ParticipantFrameStructList::const_iterator iter =
             passiveWasNotMixedList.begin();
         iter != passiveWasNotMixedList.end();
         ++iter) {
        if(mixList->size() <  *maxAudioFrameCounter + mixListStartSize) {
          mixList->push_back(FrameAndMuteInfo((*iter)->audioFrame,
                                              (*iter)->muted));
            (*mixParticipantList)[(*iter)->audioFrame->id_] =
                (*iter)->participant;
            assert(mixParticipantList->size() <=
                   kMaximumAmountOfMixedParticipants);
        } else {
            _audioFramePool->PushMemory((*iter)->audioFrame);
        }
        delete *iter;
    }
    assert(*maxAudioFrameCounter + mixListStartSize >= mixList->size());
    *maxAudioFrameCounter += mixListStartSize - mixList->size();
}

void AudioConferenceMixerImpl::GetAdditionalAudio(
    AudioFrameList* additionalFramesList) const {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                 "GetAdditionalAudio(additionalFramesList)");
    // The GetAudioFrameWithMuted() callback may result in the participant being
    // removed from additionalParticipantList_. If that happens it will
    // invalidate any iterators. Create a copy of the participants list such
    // that the list of participants can be traversed safely.
    MixerParticipantList additionalParticipantList;
    additionalParticipantList.insert(additionalParticipantList.begin(),
                                     _additionalParticipantList.begin(),
                                     _additionalParticipantList.end());

    for (MixerParticipantList::const_iterator participant =
             additionalParticipantList.begin();
         participant != additionalParticipantList.end();
         ++participant) {
        AudioFrame* audioFrame = NULL;
        if(_audioFramePool->PopMemory(audioFrame) == -1) {
            WEBRTC_TRACE(kTraceMemory, kTraceAudioMixerServer, _id,
                         "failed PopMemory() call");
            assert(false);
            return;
        }
        audioFrame->sample_rate_hz_ = _outputFrequency;
        auto ret = (*participant)->GetAudioFrameWithMuted(_id, audioFrame);
        if (ret == MixerParticipant::AudioFrameInfo::kError) {
            WEBRTC_TRACE(kTraceWarning, kTraceAudioMixerServer, _id,
                         "failed to GetAudioFrameWithMuted() from participant");
            _audioFramePool->PushMemory(audioFrame);
            continue;
        }
        if(audioFrame->samples_per_channel_ == 0) {
            // Empty frame. Don't use it.
            _audioFramePool->PushMemory(audioFrame);
            continue;
        }
        additionalFramesList->push_back(FrameAndMuteInfo(
            audioFrame, ret == MixerParticipant::AudioFrameInfo::kMuted));
    }
}

void AudioConferenceMixerImpl::UpdateMixedStatus(
    const std::map<int, MixerParticipant*>& mixedParticipantsMap) const {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                 "UpdateMixedStatus(mixedParticipantsMap)");
    assert(mixedParticipantsMap.size() <= kMaximumAmountOfMixedParticipants);

    // Loop through all participants. If they are in the mix map they
    // were mixed.
    for (MixerParticipantList::const_iterator
        participant =_participantList.begin();
        participant != _participantList.end();
         ++participant) {
        bool isMixed = false;
        for (auto it = mixedParticipantsMap.begin();
             it != mixedParticipantsMap.end();
             ++it) {
          if (it->second == *participant) {
            isMixed = true;
            break;
          }
        }
        (*participant)->_mixHistory->SetIsMixed(isMixed);
    }
}

void AudioConferenceMixerImpl::ClearAudioFrameList(
    AudioFrameList* audioFrameList) const {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                 "ClearAudioFrameList(audioFrameList)");
    for (AudioFrameList::iterator iter = audioFrameList->begin();
         iter != audioFrameList->end();
         ++iter) {
        _audioFramePool->PushMemory(iter->frame);
    }
    audioFrameList->clear();
}

bool AudioConferenceMixerImpl::IsParticipantInList(
    const MixerParticipant& participant,
    const MixerParticipantList& participantList) const {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                 "IsParticipantInList(participant,participantList)");
    for (MixerParticipantList::const_iterator iter = participantList.begin();
         iter != participantList.end();
         ++iter) {
        if(&participant == *iter) {
            return true;
        }
    }
    return false;
}

bool AudioConferenceMixerImpl::AddParticipantToList(
    MixerParticipant* participant,
    MixerParticipantList* participantList) const {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                 "AddParticipantToList(participant, participantList)");
    participantList->push_back(participant);
    // Make sure that the mixed status is correct for new MixerParticipant.
    participant->_mixHistory->ResetMixedStatus();
    return true;
}

bool AudioConferenceMixerImpl::RemoveParticipantFromList(
    MixerParticipant* participant,
    MixerParticipantList* participantList) const {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                 "RemoveParticipantFromList(participant, participantList)");
    for (MixerParticipantList::iterator iter = participantList->begin();
         iter != participantList->end();
         ++iter) {
        if(*iter == participant) {
            participantList->erase(iter);
            // Participant is no longer mixed, reset to default.
            participant->_mixHistory->ResetMixedStatus();
            return true;
        }
    }
    return false;
}

int32_t AudioConferenceMixerImpl::MixFromList(
    AudioFrame* mixedAudio,
    const AudioFrameList& audioFrameList) const {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                 "MixFromList(mixedAudio, audioFrameList)");
    if(audioFrameList.empty()) return 0;

    uint32_t position = 0;

    if (_numMixedParticipants == 1) {
      mixedAudio->timestamp_ = audioFrameList.front().frame->timestamp_;
      mixedAudio->elapsed_time_ms_ =
          audioFrameList.front().frame->elapsed_time_ms_;
    } else {
      // TODO(wu): Issue 3390.
      // Audio frame timestamp is only supported in one channel case.
      mixedAudio->timestamp_ = 0;
      mixedAudio->elapsed_time_ms_ = -1;
    }

    for (AudioFrameList::const_iterator iter = audioFrameList.begin();
         iter != audioFrameList.end();
         ++iter) {
        if(position >= kMaximumAmountOfMixedParticipants) {
            WEBRTC_TRACE(
                kTraceMemory,
                kTraceAudioMixerServer,
                _id,
                "Trying to mix more than max amount of mixed participants:%d!",
                kMaximumAmountOfMixedParticipants);
            // Assert and avoid crash
            assert(false);
            position = 0;
        }
        if (!iter->muted) {
          MixFrames(mixedAudio, iter->frame, _use_limiter);
        }

        position++;
    }

    return 0;
}

// TODO(andrew): consolidate this function with MixFromList.
int32_t AudioConferenceMixerImpl::MixAnonomouslyFromList(
    AudioFrame* mixedAudio,
    const AudioFrameList& audioFrameList) const {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                 "MixAnonomouslyFromList(mixedAudio, audioFrameList)");

    if(audioFrameList.empty()) return 0;

    for (AudioFrameList::const_iterator iter = audioFrameList.begin();
         iter != audioFrameList.end();
         ++iter) {
        if (!iter->muted) {
            MixFrames(mixedAudio, iter->frame, _use_limiter);
        }
    }
    return 0;
}

bool AudioConferenceMixerImpl::LimitMixedAudio(AudioFrame* mixedAudio) const {
    if (!_use_limiter) {
      return true;
    }

    _mixed_buffer->DeinterleaveFrom(mixedAudio);
    // _mixed_buffer->SplitIntoFrequencyBands(); // transient need this
    // _mixed_buffer->CopyLowPassToReference(); // ns need this

    /* _filter->Process(_mixed_buffer.get(), false); */
    /* _suppressor->AnalyzeCaptureAudio(_mixed_buffer.get()); */
    _limiter->AnalyzeCaptureAudio(_mixed_buffer.get());
    // Smoothly limit the mixed frame.
    /* _suppressor->ProcessCaptureAudio(_mixed_buffer.get()); */
    const int error = _limiter->ProcessCaptureAudio(_mixed_buffer.get());
    /* // Put float data in an AudioFrameView.
    AudioFrameView<float> mixing_buffer_view(_mixed_buffer->channels_f(),
                                             mixedAudio->num_channels(),
                                             mixedAudio->samples_per_channel());
    // const size_t sample_rate = mixing_buffer_view.samples_per_channel() * 1000 /
    //                            AudioConferenceMixerImpl::kProcessPeriodicityInMs;
    _limiter->SetSampleRate(mixedAudio->sample_rate_hz());
    _limiter->Process(mixing_buffer_view); */
    /* // _suppressor->Suppress(
    //     _mixed_buffer->channels_f()[0], _mixed_buffer->num_frames(),
    //     _mixed_buffer->num_channels(),
    //     _mixed_buffer->split_bands_const_f(0)[kBand0To8kHz], _mixed_buffer->num_frames_per_band(),
    //     NULL, 0,
    //     1.f, // _agc_manager->voice_probability()
    //     false); */

    // _mixed_buffer->MergeFrequencyBands(); // if split, then merge
    _mixed_buffer->InterleaveTo(mixedAudio, true);

    // And now we can safely restore the level. This procedure results in
    // some loss of resolution, deemed acceptable.
    //
    // It's possible to apply the gain in the AGC (with a target level of 0 dbFS
    // and compression gain of 6 dB). However, in the transition frame when this
    // is enabled (moving from one to two participants) it has the potential to
    // create discontinuities in the mixed frame.
    //
    // Instead we double the frame (with addition since left-shifting a
    // negative value is undefined).
    AudioFrameOperations::Add(*mixedAudio, mixedAudio);

    if(error != AudioProcessing::kNoError) {
        WEBRTC_TRACE(kTraceError, kTraceAudioMixerServer, _id,
                     "Error from AudioProcessing: %d", error);
        assert(false);
        return false;
    }
    return true;
}

// woogeen vad
int32_t AudioConferenceMixerImpl::RegisterMixerVadCallback(
        AudioMixerVadReceiver *vadReceiver,
        const uint32_t amountOf10MsBetweenCallbacks) {
    if(amountOf10MsBetweenCallbacks == 0) {
        WEBRTC_TRACE(
            kTraceWarning,
            kTraceAudioMixerServer,
            _id,
            "amountOf10MsBetweenCallbacks(%d) needs to be larger than 0",
            amountOf10MsBetweenCallbacks
            );
        return -1;
    }
    {
        rtc::CritScope cs(&_cbCrit);
        if(_vadReceiver != NULL) {
            WEBRTC_TRACE(kTraceWarning, kTraceAudioMixerServer, _id,
                         "Mixer vad callback already registered");
            return -1;
        }
        _vadReceiver  = vadReceiver;
    }
    {
        rtc::CritScope cs(&_crit);
        _amountOf10MsBetweenVadCallbacks = amountOf10MsBetweenCallbacks;
        _amountOf10MsAll = 0;
        _amountOf10MsRemainder = 0;

        _vadEnabled = true;
    }
    return 0;
}

int32_t AudioConferenceMixerImpl::UnRegisterMixerVadCallback() {
    {
        rtc::CritScope cs(&_crit);
        if(!_vadEnabled)
            return -1;

        _vadEnabled = false;
        _amountOf10MsBetweenVadCallbacks = 0;
        _amountOf10MsAll = 0;
        _amountOf10MsRemainder = 0;
    }
    {
        rtc::CritScope cs(&_cbCrit);
        _vadReceiver = NULL;
    }
    return 0;
}

uint32_t AudioConferenceMixerImpl::CombinedEnergy(
    AudioFrame* frame) const {
#if defined (_VAD_METHOD_ENERGY_)
    const bool use_energy = true, use_vad = false;
#elif defined (_VAD_METHOD_VOICE_DETECTION_)
    const bool use_energy = false, use_vad = true;
#elif defined (_VAD_METHOD_JOINT_ENERGY_VOICE_DETECTION_)
    const bool use_energy = true, use_vad = true;
#else
#error "Invalid VAD method!"
#endif
    uint32_t energy = 0;

    if (use_energy) {
        energy = CalculateEnergy(*frame);
    }
    if (use_vad) {
        int32_t id = frame->id_;
        if (_vads.find(id) == _vads.end()) {
            /* _vad_buffers[id].reset(
                new AudioBuffer(frame->sample_rate_hz(), frame->num_channels(),
                                frame->sample_rate_hz(), frame->num_channels(),
                                frame->sample_rate_hz(), frame->num_channels()));
            _vad_crits[id].reset(
                new rtc::CriticalSection());
            _vads[id].reset(
                new VoiceDetectionImpl(_vad_crits[id].get()));
            _vads[id]->Enable(true);
            _vads[id]->Initialize(frame->sample_rate_hz());
            _vads[id]->set_likelihood(VoiceDetection::kLowLikelihood);  */ 
            _vads[id].reset(
                StandaloneVad::Create());
            _vads[id]->set_mode(2); // VoiceDetection::kLowLikelihood
        }
        /* _vad_buffers[id]->DeinterleaveFrom(frame);
        _vads[id]->ProcessCaptureAudio(_vad_buffers[id].get());
        _vad_buffers[id]->InterleaveTo(frame, false);
        bool hasVoice = _vads[id]->stream_has_voice(); */
        double p[1] = {0}; // StandaloneVad::kMaxNum10msFrames
        _vads[id]->AddAudio(frame->data(), frame->samples_per_channel());
        _vads[id]->GetActivity(p, arraysize(p));

        if (!use_energy) {
            energy = p[0] * 100.0;
        } else {
            energy = (p[0] > 0.01) ? energy : 0;
        }
    }

    return energy;
}

// void AudioConferenceMixerImpl::ClearVads() {
//     /* _vads.clear();
//     _vad_crits.clear();
//     _vad_buffers.clear(); */
//     _vads.clear();
// }

void AudioConferenceMixerImpl::UpdateVadStatistics(AudioFrameList* mixList) {
    WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
            "UpdateVadStatistics: %d",
            _amountOf10MsRemainder);

    for (AudioFrameList::iterator iter = mixList->begin();
         iter != mixList->end();
         ++iter) {
        if(iter->frame->vad_activity_ == AudioFrame::kVadActive) {
            int32_t id = iter->frame->id_;
            uint32_t energy = 0;

            if (_vadParticipantEnergyList.find(id) == _vadParticipantEnergyList.end()) {
                _vadParticipantEnergyList[id] = 0;
            }

            _vadParticipantEnergyList[id] += CombinedEnergy(iter->frame);

            WEBRTC_TRACE(kTraceStream, kTraceAudioMixerServer, _id,
                    "###VadStatistics id(0x%-8x), energy(%12u), allEnergy(%12ld)",
                    id, energy, _vadParticipantEnergyList[id]);
        }
    }
}

void AudioConferenceMixerImpl::SetMultipleInputs(bool enable)
{
    WEBRTC_TRACE(kTraceInfo, kTraceAudioMixerServer, _id,
            "Support multiple inputs: %d",
            enable);

    _supportMultipleInputs = enable;
}

}  // namespace webrtc
