/*
grrrr.org 2025:

This header depends on the base templates defined in "usb_audio.h"

It implements the OpenAudio_ArduinoLibrary-compatible USB audio interface.
*/

#pragma once

#include <AudioStream_F32.h>

#include "usb_audio.h"

// Specialization for AudioStream_F32
template<>
class AudioUSB_Base<AudioStream_F32>:
	public AudioStream_F32
{
protected:
	typedef AudioStream_F32 StreamClass;
	typedef float32_t sample_t;
	typedef ::audio_block_f32_t audio_block_t;

	// define various compatibility functions
	static AudioUSB_Base::audio_block_t *allocate() { return StreamClass::allocate_f32(); }
	static void release(AudioUSB_Base::audio_block_t *block) { StreamClass::release(block); }
	audio_block_t *receiveReadOnly(unsigned int index = 0) { return StreamClass::receiveReadOnly_f32(index); }
	audio_block_t *receiveWritable(unsigned int index = 0) { return StreamClass::receiveWritable_f32(index); }
	void transmit(AudioUSB_Base::audio_block_t *block, unsigned char index = 0) { StreamClass::transmit(block, index); }
	static int blocklength(const AudioUSB_Base::audio_block_t *block) { return block->length; }

public:
	AudioUSB_Base(unsigned char ninput, AudioUSB_Base::audio_block_t **iqueue): StreamClass(ninput, iqueue) {}

protected:
	static int sample_to_buffer(sample_t *dst, const uint8_t *src)
	{
	#if AUDIO_USB_FORMAT == 1 // PCM
		#if AUDIO_SUBSLOT_SIZE>=2 && AUDIO_SUBSLOT_SIZE<=4
			// USB PCM data is always signed
			union {
				int32_t i32;
				uint8_t u8[4];
			} tmp;
			constexpr auto scale = 1<<(sizeof(tmp.i32)*8-1);
			tmp.i32 = 0;
			for(int k = 0; k < AUDIO_SUBSLOT_SIZE; ++k, ++src)
				tmp.u8[k+(4-AUDIO_SUBSLOT_SIZE)] = *src;
			// convert to float
			*dst = tmp.i32*(1.f/float32_t(scale));
		#else
			#error AUDIO_SUBSLOT_SIZE invalid
		#endif
	#elif AUDIO_USB_FORMAT == 4 // IEEE_FLOAT
		*dst = *(const float32_t *)(src);
	#else
		#error AUDIO_USB_FORMAT invalid
	#endif
		return AUDIO_SUBSLOT_SIZE;
	}

	static int sample_from_buffer(uint8_t *dst, const sample_t *src)
	{
	#if AUDIO_USB_FORMAT == 1 // PCM
		#if AUDIO_SUBSLOT_SIZE>=2 && AUDIO_SUBSLOT_SIZE<=4
			union {
				int32_t i32;
				uint8_t u8[4];
			} tmp;
			constexpr auto scale = 1<<(sizeof(tmp.i32)*8-1);
			constexpr auto fmin = -1.f;
			constexpr auto fmax = float((scale-1.)/scale);
			// we need to pre-clip incoming float data
			tmp.i32 = max(min(*src, fmax), fmin)*float(scale);
			for(int k = 0; k < AUDIO_SUBSLOT_SIZE; ++k, ++dst)
				*dst = tmp.u8[k+(4-AUDIO_SUBSLOT_SIZE)];
		#else
			#error AUDIO_SUBSLOT_SIZE invalid
		#endif
	#elif AUDIO_USB_FORMAT == 4 // IEEE_FLOAT
		*(float32_t *)dst = *src;
	#else
		#error AUDIO_USB_FORMAT invalid
	#endif
		return AUDIO_SUBSLOT_SIZE;
	}
};

using AudioInputUSB_F32 = AudioInputUSB_Proto<AudioStream_F32>;
using AudioOutputUSB_F32 = AudioOutputUSB_Proto<AudioStream_F32>;

