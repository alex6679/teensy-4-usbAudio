/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once


#include "usb_desc.h"
#include "util/LastCall.h"
#ifdef AUDIO_INTERFACE

#define FEATURE_MAX_VOLUME 0xFF  // volume accepted from 0 to 0xFF
#define TARGET_RX_BUFFER_TIME_S 0.0018f	//targeted buffered time (latency) in seconds

#ifdef __cplusplus
extern "C" {
#endif
extern void usb_audio_configure();
extern uint16_t usb_audio_receive_buffer[];
extern uint8_t usb_audio_transmit_buffer[];
extern uint32_t usb_audio_sync_feedback;
extern uint8_t usb_audio_receive_setting;
extern uint8_t usb_audio_transmit_setting;
extern void usb_audio_receive_callback(unsigned int len);
extern unsigned int usb_audio_transmit_callback(void);
extern int usb_audio_set_feature(void *stp, uint8_t *buf);
extern int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen);
#ifdef __cplusplus
}
#endif

// audio features supported
struct usb_audio_features_struct {
  int change;  // set to 1 when any value is changed
  int mute;    // 1=mute, 0=unmute
  int volume;  // volume from 0 to FEATURE_MAX_VOLUME, maybe should be float from 0.0 to 1.0
};

#ifdef __cplusplus
#include "AudioStream.h"
class AudioInputUSB : public AudioStream
{
public:
	struct Status {
		uint32_t usb_audio_underrun_count;
		uint32_t usb_audio_overrun_count;
		uint32_t audio_memory_underrun_count;
		float target_num_buffered_samples;
		uint16_t num_transmitted_channels;		//might be smaller than expected in case the 12Mbit/s bandwidth limits the number of channels
		uint16_t ring_buffer_size;
		uint16_t usb_rx_tx_buffer_size;
		uint16_t bInterval_uS;
		bool receivingData;
	};

	AudioInputUSB(float kp =400.f,float ki =.2f ) 
		: AudioStream(0, NULL), _kp(kp), _ki(ki) 
		{ begin(); }
	virtual void update(void);
	void begin(void);
	float getBufferedSamples() const;
	float getBufferedSamplesSmooth() const;
	float getRequestedSamplingFrequ() const;
	Status getStatus() const;
	friend void usb_audio_receive_callback(unsigned int len);
	friend int usb_audio_set_feature(void *stp, uint8_t *buf);
	friend int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen);
	static struct usb_audio_features_struct features;
	float volume(void) {
		if (features.mute) return 0.0;
		return (float)(features.volume) * (1.0 / (float)FEATURE_MAX_VOLUME);
	}
private:
	static bool update_responsibility;
	uint32_t _bufferedSamples=0;
	float _kp =400.f;
	float _ki =.2f;
	float _bufferedSamplesSmooth=0;
	bool _streaming= false;
	LastCall<50> _lastCallUpdate;
	static bool setBlocksQuite(uint32_t noBlocks);
	static bool allocateChannels(uint16_t idx);
	static void resetBuffer(double updateCurrentSmooth);
	static bool isBufferReady();
	static void tryIncreaseIdxIncoming(uint16_t& count);
	static void releaseBlocks(uint16_t bufferIdx);
};

#if USB_AUDIO_NO_CHANNELS_480 >= 4
class AudioInputUSBQuad : public AudioInputUSB { public: AudioInputUSBQuad(float kp =400.f,float ki =.2f) : AudioInputUSB(kp, ki) {} };
#if USB_AUDIO_NO_CHANNELS_480 >= 6
class AudioInputUSBHex : public AudioInputUSB { public: AudioInputUSBHex(float kp =400.f,float ki =.2f) : AudioInputUSB(kp, ki) {} };
#if USB_AUDIO_NO_CHANNELS_480 >= 8
class AudioInputUSBOct : public AudioInputUSB { public: AudioInputUSBOct(float kp =400.f,float ki =.2f) : AudioInputUSB(kp, ki) {} };
#endif // USB_AUDIO_NO_CHANNELS_480 >= 8
#endif // USB_AUDIO_NO_CHANNELS_480 >= 6
#endif // USB_AUDIO_NO_CHANNELS_480 >= 4


#define TARGET_TX_BUFFER_TIME_S 0.0035f	//targeted buffered time (latency) in seconds
class AudioOutputUSB : public AudioStream
{
public:
	enum BufferState{ready, full, overrun};
	struct Status {
		uint32_t usb_audio_underrun_count;
		uint32_t usb_audio_overrun_count;
		float target_num_buffered_samples;
		uint16_t num_transmitted_channels;
		uint16_t ring_buffer_size;
		uint16_t usb_rx_tx_buffer_size;
		uint16_t bInterval_uS;
		uint32_t num_skipped_Samples;
		uint32_t num_padded_Samples;
		bool transmittingData;
	};
	AudioOutputUSB(void) : AudioStream(USB_AUDIO_MAX_NO_CHANNELS, inputQueueArray) { begin(); }
	AudioOutputUSB(int nch) : AudioStream(nch, inputQueueArray) { begin(); }
	virtual void update(void);
	void begin(void);
	friend unsigned int usb_audio_transmit_callback(void);
	float getBufferedSamples() const;
	float getBufferedSamplesSmooth() const;
	Status getStatus() const;
private:
	static void releaseBlocks(uint16_t bufferIdx);
	static void tryIncreaseIdxTransmission(uint16_t& tBIdx, uint16_t& offset);
	static bool update_responsibility;
	bool _streaming= false;
	LastCall<50> _lastCallUpdate;
	static double updateCurrentSmooth;
	static double updateCurrentSmoothAtOverrun;
	static uint16_t outgoing_count;
	audio_block_t *inputQueueArray[USB_AUDIO_MAX_NO_CHANNELS];
};

#if USB_AUDIO_NO_CHANNELS_480 >= 4
class AudioOutputUSBQuad : public AudioOutputUSB { public: AudioOutputUSBQuad(void) : AudioOutputUSB(4) {} };
#if USB_AUDIO_NO_CHANNELS_480 >= 6
class AudioOutputUSBHex : public AudioOutputUSB { public: AudioOutputUSBHex(void) : AudioOutputUSB(6) {} };
#if USB_AUDIO_NO_CHANNELS_480 >= 8
class AudioOutputUSBOct : public AudioOutputUSB { public: AudioOutputUSBOct(void) : AudioOutputUSB(8) {} };
#endif // USB_AUDIO_NO_CHANNELS_480 >= 8
#endif // USB_AUDIO_NO_CHANNELS_480 >= 6
#endif // USB_AUDIO_NO_CHANNELS_480 >= 4

#endif // __cplusplus

#endif // AUDIO_INTERFACE
