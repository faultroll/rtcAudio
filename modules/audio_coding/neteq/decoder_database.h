/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_AUDIO_CODING_NETEQ_DECODER_DATABASE_H_
#define WEBRTC_MODULES_AUDIO_CODING_NETEQ_DECODER_DATABASE_H_

#include "rtc_base/scoped_refptr.h"
#include "rtc_base/constructor_magic.h"
// #include "modules/audio_coding/neteq/audio_decoder_impl.h"
#include "modules/audio_coding/neteq/neteq_decoder_enum.h"
#include "modules/audio_coding/neteq/packet.h"
#include "modules/audio_coding/codecs/cng/webrtc_cng.h"

namespace webrtc {

// Forward declaration.
class AudioDecoder;
class AudioDecoderFactory; // A factory that creates AudioDecoders.

class DecoderDatabase {
 public:
  enum DatabaseReturnCodes {
    kOK = 0,
    kInvalidRtpPayloadType = -1,
    kCodecNotSupported = -2,
    kInvalidSampleRate = -3,
    kDecoderExists = -4,
    kDecoderNotFound = -5,
    kInvalidPointer = -6
  };

  // Maximum value for 8 bits, and an invalid RTP payload type (since it is
  // only 7 bits).
  static const uint8_t kRtpPayloadTypeError = 0xFF;

  DecoderDatabase(
      const rtc::scoped_refptr<AudioDecoderFactory>& decoder_factory);

  virtual ~DecoderDatabase() = 0;

  // Returns true if the database is empty.
  virtual bool Empty() const = 0;

  // Returns the number of decoders registered in the database.
  virtual int Size() const = 0;

  // Resets the database, erasing all registered payload types, and deleting
  // any AudioDecoder objects that were not externally created and inserted
  // using InsertExternal().
  virtual void Reset() = 0;

  // Registers |rtp_payload_type| as a decoder of type |codec_type|. The |name|
  // is only used to populate the name field in the DecoderInfo struct in the
  // database, and can be arbitrary (including empty). Returns kOK on success;
  // otherwise an error code.
  virtual int RegisterPayload(uint8_t rtp_payload_type,
                              NetEqDecoder codec_type,
                              const char* name) = 0;

  // Registers an externally created AudioDecoder object, and associates it
  // as a decoder of type |codec_type| with |rtp_payload_type|.
  virtual int InsertExternal(uint8_t rtp_payload_type,
                             NetEqDecoder codec_type,
                             const char* codec_name,
                             AudioDecoder* decoder) = 0;

  // Removes the entry for |rtp_payload_type| from the database.
  // Returns kDecoderNotFound or kOK depending on the outcome of the operation.
  virtual int Remove(uint8_t rtp_payload_type) = 0;

  // Remove all entries.
  virtual void RemoveAll() = 0;

  // Sets the active decoder to be |rtp_payload_type|. If this call results in a
  // change of active decoder, |new_decoder| is set to true. The previous active
  // decoder's AudioDecoder object is deleted.
  virtual int SetActiveDecoder(uint8_t rtp_payload_type, bool* new_decoder) = 0;

  // Returns the current active decoder, or NULL if no active decoder exists.
  virtual AudioDecoder* GetActiveDecoder() const = 0;

  // Sets the active comfort noise decoder to be |rtp_payload_type|. If this
  // call results in a change of active comfort noise decoder, the previous
  // active decoder's AudioDecoder object is deleted.
  virtual int SetActiveCngDecoder(uint8_t rtp_payload_type) = 0;

  // Returns the current active comfort noise decoder, or NULL if no active
  // comfort noise decoder exists.
  virtual ComfortNoiseDecoder* GetActiveCngDecoder() const = 0;

  // The following are utility methods: they will look up DecoderInfo through
  // GetDecoderInfo and call the respective method on that info object, if it
  // exists.

  // Returns a pointer to the AudioDecoder object associated with
  // |rtp_payload_type|, or NULL if none is registered. If the AudioDecoder
  // object does not exist for that decoder, the object is created.
  virtual AudioDecoder* GetDecoder(uint8_t rtp_payload_type) const = 0;

  // Returns if |rtp_payload_type| is registered with a format named |name|.
  virtual bool IsType(uint8_t rtp_payload_type, const char* name) const = 0;

  // Returns true if |rtp_payload_type| is registered as comfort noise.
  virtual bool IsComfortNoise(uint8_t rtp_payload_type) const = 0;

  // Returns true if |rtp_payload_type| is registered as DTMF.
  virtual bool IsDtmf(uint8_t rtp_payload_type) const = 0;

  // Returns true if |rtp_payload_type| is registered as RED.
  virtual bool IsRed(uint8_t rtp_payload_type) const = 0;

  virtual int SampleRateHz(uint8_t rtp_payload_type) const = 0;

  // Returns kOK if all packets in |packet_list| carry payload types that are
  // registered in the database. Otherwise, returns kDecoderNotFound.
  virtual int CheckPayloadTypes(const PacketList& packet_list) const = 0;

  RTC_DISALLOW_COPY_AND_ASSIGN(DecoderDatabase);
};

}  // namespace webrtc
#endif  // WEBRTC_MODULES_AUDIO_CODING_NETEQ_DECODER_DATABASE_H_
