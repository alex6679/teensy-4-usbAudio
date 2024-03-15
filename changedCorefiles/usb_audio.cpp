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

#include <Arduino.h>
#include "usb_dev.h"
#include "usb_audio.h"
#include "debug/printf.h"

#include <string>

#ifdef AUDIO_INTERFACE
namespace {	
	
	//variables used by AudioInputUSB and AudioOutputUSB ==================
  #if (USB_AUDIO_NO_CHANNELS_12*AUDIO_NUM_SUBFRAMES_PER_POLLING_12) < (USB_AUDIO_NO_CHANNELS_480*AUDIO_NUM_SUBFRAMES_PER_POLLING_480)
    #define AUDIO_TX_SIZE         AUDIO_RX_SIZE_480
    #define AUDIO_RX_SIZE         AUDIO_RX_SIZE_480
  #else
    #define AUDIO_TX_SIZE         AUDIO_RX_SIZE_12
    #define AUDIO_RX_SIZE         AUDIO_RX_SIZE_12
  #endif

	constexpr float blockDuration = AUDIO_BLOCK_SAMPLES/float(AUDIO_SAMPLE_RATE);
	uint16_t noTransmittedChannels=0;	//depending if usb_high_speed if true this is either USB_AUDIO_MAX_NO_CHANNELS or 2 as fall-back strategy
	float audioPollingIntervalSec=0;
	uint16_t audioPollingIntervaluS =0;
	uint32_t noSamplesPerPollingInterval=0;
	constexpr uint32_t samplingRate=uint32_t(AUDIO_SAMPLE_RATE);
	float expectedIsrIntervalCycles;

	//======================================================================

	LastCall<7> lastCallReceiveIsr;
	float sumDiff=0.;
	float lastDiff=0.;
	
	//no buffered blocks computation
	//
	// +3:
	// +1 we round the number of needed blocks up
	// +1 because we store the block that is ready for transmission in the buffer
	// +1 just to be able have one block in reserve (prevents buffer overflows and comes at more or less no additional cost (at normal operation no additional latency + no additional memory needed))
	// *2 because to be symmetrically protected against buffer overflow (a high TARGET_RX_BUFFER_TIME_S only prevents buffer underflows)
	constexpr uint16_t ringRxBufferSize = uint16_t(TARGET_RX_BUFFER_TIME_S / blockDuration) *2 +3;	
	
	audio_block_t* rxBuffer[ringRxBufferSize][USB_AUDIO_MAX_NO_CHANNELS];
	constexpr float targetNumRxBufferedSamples = TARGET_RX_BUFFER_TIME_S*AUDIO_SAMPLE_RATE;
	bool rxBufferReady= false;	//used to indicate if the reset after e.g. a buffer overrun is completed
	volatile bool rxBufferOverrun = false;			//changed in usb_audio_receive_callback
	volatile uint32_t rxMemoryUnderrunCounter=0;	//changed in usb_audio_receive_callback
	volatile uint16_t incoming_rx_bIdx=0;			//changed in usb_audio_receive_callback
	volatile uint16_t rxIncoming_count=0;			//changed in usb_audio_receive_callback
	volatile uint8_t receive_flag;					//changed in usb_audio_receive_callback

	uint16_t transmit_rx_bIdx=0;
	uint32_t feedback_accumulator;
	uint32_t feedback_accumulator_default;

	volatile uint32_t rxUsb_audio_underrun_count;
	volatile uint32_t rxUsb_audio_overrun_count;
		
	float getNumBufferedRxSamples(uint16_t incomingIdx, uint16_t transmitIdx, uint16_t incomingCount){
		float bufferedSamples= incomingCount;
		if(incomingIdx > transmitIdx+1 ){
			bufferedSamples+=(incomingIdx-(transmitIdx+1))*AUDIO_BLOCK_SAMPLES;
		}
		else if(incomingIdx < transmitIdx){
			bufferedSamples += incomingIdx*AUDIO_BLOCK_SAMPLES;
			if(transmitIdx +1 < ringRxBufferSize){
				bufferedSamples += (ringRxBufferSize-(transmitIdx +1))*AUDIO_BLOCK_SAMPLES;
			}
		}
		return bufferedSamples;
	}
}

bool AudioInputUSB::update_responsibility;

struct usb_audio_features_struct AudioInputUSB::features = {0,0,FEATURE_MAX_VOLUME/2};

extern volatile uint8_t usb_high_speed;
static void rx_event(transfer_t *t);
static void tx_event(transfer_t *t);

/*static*/ transfer_t rx_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t sync_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t tx_transfer __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[AUDIO_RX_SIZE] __attribute__ ((aligned(32)));
DMAMEM uint32_t usb_audio_sync_feedback __attribute__ ((aligned(32)));

uint8_t usb_audio_receive_setting=0;	//set in usb.c
uint8_t usb_audio_transmit_setting=0;	//set in usb.c
uint8_t usb_audio_sync_nbytes;
uint8_t usb_audio_sync_rshift;


static void rx_event(transfer_t *t)
{
	if (t) {
		int len = AUDIO_RX_SIZE - ((rx_transfer.status >> 16) & 0x7FFF);
		//printf("rx %u\n", len);
		usb_audio_receive_callback(len);
	}
	usb_prepare_transfer(&rx_transfer, rx_buffer, AUDIO_RX_SIZE, 0);
	arm_dcache_delete(&rx_buffer, AUDIO_RX_SIZE);
	usb_receive(AUDIO_RX_ENDPOINT, &rx_transfer);
}

static void sync_event(transfer_t *t)
{
	// USB 2.0 Specification, 5.12.4.2 Feedback, pages 73-75
	//printf("sync %x\n", sync_transfer.status); // too slow, can't print this much
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&sync_transfer, &usb_audio_sync_feedback, usb_audio_sync_nbytes, 0);
	arm_dcache_flush(&usb_audio_sync_feedback, usb_audio_sync_nbytes);
	usb_transmit(AUDIO_SYNC_ENDPOINT, &sync_transfer);
}

AudioInputUSB::Status AudioInputUSB::getStatus() const{
	AudioInputUSB::Status status;
	NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
	status.usb_audio_underrun_count = rxUsb_audio_underrun_count;
	status.usb_audio_overrun_count = rxUsb_audio_overrun_count;
	status.audio_memory_underrun_count = rxMemoryUnderrunCounter;
	status.target_num_buffered_samples = targetNumRxBufferedSamples;
	status.num_transmitted_channels = noTransmittedChannels;
	status.ring_buffer_size = ringRxBufferSize;
	status.usb_rx_tx_buffer_size = AUDIO_RX_SIZE;
	status.receivingData=_streaming;
	status.bInterval_uS = audioPollingIntervaluS;
	NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	return status;
}

void AudioInputUSB::releaseBlocks(uint16_t bufferIdx){
	for (uint16_t j =0; j< noTransmittedChannels; j++){
		if(rxBuffer[bufferIdx][j]){
			AudioStream::release(rxBuffer[bufferIdx][j]);
			rxBuffer[bufferIdx][j]=NULL;
		}
	}	
}

bool AudioInputUSB::setBlocksQuite(uint32_t noBlocks){
	bool allocationSuccessful=true;
	for (uint16_t i =0; i< ringRxBufferSize; i++){
		for (uint16_t j =0; j< noTransmittedChannels; j++){
			if(i < noBlocks){
				if(!rxBuffer[i][j]){
					rxBuffer[i][j] = AudioStream::allocate();
				}
				if(rxBuffer[i][j]){
					memset(rxBuffer[i][j]->data, 0, AUDIO_BLOCK_SAMPLES*sizeof(rxBuffer[i][j]->data[0]));
				}
				else {
					//allocation failed
					allocationSuccessful=false;
					break;
				}
			}
			else if(rxBuffer[i][j]){
				AudioStream::release(rxBuffer[i][j]);
				rxBuffer[i][j]=NULL;
			}
		}
	}
	if(!allocationSuccessful){
		for (uint16_t i =0; i< ringRxBufferSize; i++){
			releaseBlocks(i);
		}	
	}
	return allocationSuccessful;
}

bool AudioInputUSB::allocateChannels(uint16_t idx){
	for (uint16_t i =0; i< noTransmittedChannels; i++){
		if(!rxBuffer[idx][i]){
			rxBuffer[idx][i] = AudioStream::allocate();
		}
		if(!rxBuffer[idx][i]){
			//allocation failed			
			for (uint32_t j =0; j< i; j++){
				AudioStream::release(rxBuffer[idx][j]);
				rxBuffer[idx][j]=NULL;
			}
			return false;
		}
	}
	return true;
}

void AudioInputUSB::begin(void)
{
	rxIncoming_count = 0;
	for (uint16_t i =0; i< ringRxBufferSize; i++){
		for (uint16_t j =0; j< USB_AUDIO_MAX_NO_CHANNELS; j++){
			rxBuffer[i][j]=NULL;
		}
	}
	receive_flag = 0;
	_streaming=false;
	// update_responsibility = update_setup();
	// TODO: update responsibility is tough, partly because the USB
	// interrupts aren't sychronous to the audio library block size,
	// but also because the PC may stop transmitting data, which
	// means we no longer get receive callbacks from usb.c
	update_responsibility = false;
	_lastCallUpdate.reset(blockDuration*F_CPU_ACTUAL);
}

#if AUDIO_SUBSLOT_SIZE==2
static void copy_to_buffers(const uint8_t *src, audio_block_t *blocks[USB_AUDIO_MAX_NO_CHANNELS], unsigned int count, unsigned int len) {
	const uint16_t *src16Bit =(const uint16_t *)src;
	for (uint32_t i =0; i< len; i++){
		for (uint16_t j =0; j< noTransmittedChannels; j++){
			blocks[j]->data[count +i]=*src16Bit++;
		}
	}
}
#endif

#if AUDIO_SUBSLOT_SIZE==3
static void copy_to_buffers(const uint8_t *src, audio_block_t *blocks[USB_AUDIO_MAX_NO_CHANNELS], unsigned int count, unsigned int len) {
	for (uint32_t i =0; i< len; i++){
		for (uint16_t j =0; j< noTransmittedChannels; j++){
			++src;
			blocks[j]->data[count +i]=(*src++);
			blocks[j]->data[count +i] |=(*src++)<<8;
		}
	}
}
#endif

void AudioInputUSB::tryIncreaseIdxIncoming(uint16_t& count){
	uint16_t idx = (incoming_rx_bIdx+1)%ringRxBufferSize;
	if(idx == transmit_rx_bIdx){
		return;
	}
	incoming_rx_bIdx=idx;
	if(!AudioInputUSB::allocateChannels(incoming_rx_bIdx)){
		rxMemoryUnderrunCounter++;
		return;
	}
	count =0;	
}

// Called from the USB interrupt when an isochronous packet arrives
// we must completely remove it from the receive buffer before returning
//
#if 1
void usb_audio_receive_callback(unsigned int len)
{	
	uint32_t t = ARM_DWT_CYCCNT;
	lastCallReceiveIsr.addCall(t);
	receive_flag = 1;
	if(!rxBufferReady){
		return;
	}
	
	len /= (AUDIO_SUBSLOT_SIZE* noTransmittedChannels); // 1 sample = AUDIO_SUBSLOT_SIZE bytes times noTransmittedChannels channels
	uint16_t count =  rxIncoming_count;
	const uint8_t *data = rx_buffer;
	while (len > 0){
		if(count == AUDIO_BLOCK_SAMPLES){
			AudioInputUSB::tryIncreaseIdxIncoming(count);
		}
		if(count == AUDIO_BLOCK_SAMPLES){
			//we were not able to increase te buffer index in the ring index
			rxBufferOverrun=true;
			rxIncoming_count=count;
			return;
		}
		uint32_t avail = AUDIO_BLOCK_SAMPLES - count;
		uint32_t numToCopy = min(avail, len);
		copy_to_buffers(data, rxBuffer[incoming_rx_bIdx], count, numToCopy);
		data+=noTransmittedChannels*numToCopy *AUDIO_SUBSLOT_SIZE;
		count+=numToCopy;
		len-=numToCopy;
	}
	if(count == AUDIO_BLOCK_SAMPLES){
		//maybe we can already provide the current incoming_rx_bIdx block and prevent a buffer underflow
		//if tryIncreaseIdxIncoming fails now, there is still the chance that it will succeed at the next isr call
		AudioInputUSB::tryIncreaseIdxIncoming(count);
	}
	rxIncoming_count=count;
}
#endif

float AudioInputUSB::getBufferedSamples() const{
	NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
	float b = _bufferedSamples;
	NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	return b;
}
float AudioInputUSB::getBufferedSamplesSmooth() const{
	NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
	float b = _bufferedSamplesSmooth;
	NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	return b;
}
float AudioInputUSB::getRequestedSamplingFrequ() const{	
	NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
	float fs = feedback_accumulator/(audioPollingIntervalSec*0x1000000);
	NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	return fs;
}
bool AudioInputUSB::isBufferReady() {
	if(transmit_rx_bIdx == incoming_rx_bIdx){
		return false;
	}
	for (uint16_t i =0; i< noTransmittedChannels; i++){
		if(!rxBuffer[transmit_rx_bIdx][i]){
			return false;
		}
	}
	return true;
}
void AudioInputUSB::resetBuffer(double updateCurrentSmooth){
	//resetBuffer should be called from the update function
	//Here we prepare the buffer for the transmission after a buffer under- or overflow or at the beginning of a stream

	//first we estimate when the last usb samples arrived
	double timeSinceLastUSBPaket=0.;
	History<7> historyIsr = lastCallReceiveIsr.getHistory();	//important: a new history is needed that is consistent with incoming_rx_bIdx
	if(historyIsr.valid){
		//historyUpdate.valid is always true
		double lastIsrSmooth = lastCallReceiveIsr.getLastCall<2>(historyIsr, expectedIsrIntervalCycles);
		timeSinceLastUSBPaket = toUInt32Range(updateCurrentSmooth - lastIsrSmooth);
		timeSinceLastUSBPaket /= F_CPU_ACTUAL; //to seconds
		if(timeSinceLastUSBPaket > 1.5f*audioPollingIntervalSec || timeSinceLastUSBPaket < -0.5f){
			//normally this should not happen
			timeSinceLastUSBPaket = 0.5f*blockDuration;
		}
	}
	else {
		timeSinceLastUSBPaket = expectedIsrIntervalCycles*0.5/F_CPU_ACTUAL;
	}
	//=====================================================

	//now we set incoming_rx_bIdx, transmit_rx_bIdx and incoming_count
	float resetTimeSec = TARGET_RX_BUFFER_TIME_S+blockDuration-timeSinceLastUSBPaket;	//+blockDuration because we want to transmit one block after this function
	uint32_t resetSamples = uint32_t(resetTimeSec*AUDIO_SAMPLE_RATE);
	uint32_t noBufferedBlocks = resetSamples/AUDIO_BLOCK_SAMPLES;
	uint32_t count = resetSamples - noBufferedBlocks *AUDIO_BLOCK_SAMPLES;
	uint32_t bufferIdx = noBufferedBlocks;

	//We need to set incoming_count to back to the target value.
	//Otherwise we will nearly have an overrun at the next update call.
	//This would make it very difficult for the controller to work
	rxIncoming_count = count;
	incoming_rx_bIdx = uint16_t(bufferIdx);
	transmit_rx_bIdx =0;
}
void AudioInputUSB::update(void)
{	
	//update time measurement of update calls
	uint32_t clockCount = ARM_DWT_CYCCNT;
	_lastCallUpdate.addCall(clockCount);
	History<50> historyUpdate = _lastCallUpdate.getHistory();
	double updateCurrentSmooth= _lastCallUpdate.getLastCall<20>(historyUpdate, blockDuration*F_CPU_ACTUAL);	
	//=======================================

	//get all information related to the usb receive isr
	__disable_irq();
	History<7> historyIsr = lastCallReceiveIsr.getHistory();
	bool bufferUnderflow = !isBufferReady();
	rxBufferReady = !bufferUnderflow && !rxBufferOverrun;
	if(rxBufferOverrun){
		rxUsb_audio_overrun_count++;
	}
	uint16_t ic = rxIncoming_count;
	uint16_t iIdx = incoming_rx_bIdx;
	uint8_t f = receive_flag;
	receive_flag = 0;
	__enable_irq();
	//=======================================
	if(_streaming && !f){
		_streaming=false;
		//sumDiff+=_kp/_ki*lastDiff;	//last diff will be zero because we set incoming_count to the target -> we add lastDiff to sumDiff
		//feedback_accumulator = uint32_t(feedback_accumulator_default);
		lastCallReceiveIsr.reset(expectedIsrIntervalCycles);
		sumDiff = 0.;
		feedback_accumulator = feedback_accumulator_default;
	}
	//if !rxBufferReady we encountered an under- or overflow -> we reset the buffer positions
	if ((_streaming && !rxBufferReady) ||	//we are already streaming but encounter a buffer over- or underrun
		(!_streaming && f))	//we just start streaming -> we reset the buffer (fill it with the targeted number of samples)
	{
		_streaming = true;	//this is maybe the start of a stream
		if(bufferUnderflow && f){
			//we count under-runs only if we currently receive audio data from the host (f!=0)
			rxUsb_audio_underrun_count++;
		}
		float resetTimeSec = TARGET_RX_BUFFER_TIME_S+blockDuration;	//+blockDuration because we want to transmit one block after this function
		uint32_t resetSamples = uint32_t(resetTimeSec*AUDIO_SAMPLE_RATE);
		uint32_t noBufferedBlocks = resetSamples/AUDIO_BLOCK_SAMPLES +1;
		if(setBlocksQuite(noBufferedBlocks)){
			__disable_irq();
			//this must all happen in one un-interrupted block
			resetBuffer(updateCurrentSmooth);	
			rxBufferReady=true;
			rxBufferOverrun=false;
			bufferUnderflow=false;
			ic = rxIncoming_count;
			iIdx = incoming_rx_bIdx;
			//we do not update c here, since it should be consistent with historyIsr
			__enable_irq();
		}
		else {	
			//can only happen if we run out of audio blocks/memory		
			_streaming=false;
			rxMemoryUnderrunCounter++;
		}
	}
	//=======================================
	
	//if there was an receive event, we update the feedback for the usb host
	// Important: first compute the buffered samples before the block transmission and update of transmit_rx_bIdx below!!
	if (_streaming) {
		//we compute the mismatch of the the targeted number of buffered samples and the actual buffered samples
		float lastIsrSmooth = (float)lastCallReceiveIsr.getLastCall<2>(historyIsr, expectedIsrIntervalCycles);
		float timeSinceLastIsr = (float)toUInt32Range(updateCurrentSmooth - lastIsrSmooth);			
		timeSinceLastIsr /= F_CPU_ACTUAL; //to seconds
		
		_bufferedSamples= getNumBufferedRxSamples(iIdx, transmit_rx_bIdx, ic);
		_bufferedSamplesSmooth=_bufferedSamples + timeSinceLastIsr * AUDIO_SAMPLE_RATE;

		float diff= targetNumRxBufferedSamples -_bufferedSamplesSmooth;
		if (abs(lastDiff) <= abs(diff)){
			//we only add the current diff to the sum, if we the difference is not already decreasing
			sumDiff +=diff;
		}
		lastDiff = diff;
		//Todo: feedback_accumulator should never request more samples than there is space in the receive buffer
		feedback_accumulator = uint32_t(feedback_accumulator_default + double(_kp*diff)  + double(_ki*sumDiff) +0.5);
		//========================================================================================================
		
		//if buffer is ready, we transmit all channels and increase the transmit index in the ring buffer
		for (uint16_t i =0; i<noTransmittedChannels; i++){
			transmit(rxBuffer[transmit_rx_bIdx][i], i);
			release(rxBuffer[transmit_rx_bIdx][i]);
			rxBuffer[transmit_rx_bIdx][i]=NULL;
		}
		__disable_irq();
		transmit_rx_bIdx = (transmit_rx_bIdx+1)%ringRxBufferSize;	//it is ok if transmit_rx_bIdx==incoming_rx_bIdx here
		__enable_irq();
		//=========================================
	}
	
}

















#if 1

namespace {	
	constexpr uint16_t ringTxBufferSize = uint16_t(TARGET_TX_BUFFER_TIME_S / blockDuration) *2 +3;	
	//uint32_t lastIsr=0;
	audio_block_t* txBuffer[ringTxBufferSize][USB_AUDIO_MAX_NO_CHANNELS];
	uint16_t incoming_tx_bIdx=0;
	volatile uint16_t transmit_tx_bIdx=0;			//changed in usb_audio_transmit_callback
	volatile AudioOutputUSB::BufferState txBufferState = AudioOutputUSB::ready;			//0: buffer can be used, 1: buffer is full, 2: buffer overrun changed in usb_audio_transmit_callback	
	volatile float bufferedTxSamples=0.f;			//changed in usb_audio_transmit_callback
	volatile float virtualSamples=0.f;			//changed in usb_audio_transmit_callback
	volatile float bufferedTxSamplesSmooth=0.f;		//changed in usb_audio_transmit_callback
	volatile uint8_t transmit_flag;					//changed in usb_audio_transmit_callback

	volatile uint32_t txUsb_audio_underrun_count =0;
	volatile uint32_t txUsb_audio_overrun_count =0;
	
	volatile uint32_t num_skipped_Samples=0;
	volatile uint32_t num_padded_Samples=0;
	
	LastCall<7> lastCallTransmitIsr;
	constexpr float targetNumTxBufferedSamples = TARGET_TX_BUFFER_TIME_S*AUDIO_SAMPLE_RATE;

	float getNumBufferedTxSamples(AudioOutputUSB::BufferState txBufferState, uint32_t target, uint16_t incomingIdx, uint16_t transmitIdx, uint16_t outgoingCount){
		float bufferedSamples= AUDIO_BLOCK_SAMPLES-outgoingCount;
		if(txBufferState > AudioOutputUSB::ready){
			//buffer full or overrund
			bufferedSamples +=(AUDIO_BLOCK_SAMPLES *(ringTxBufferSize-1));
			return bufferedSamples -target;	//-target because it's assumed that target number of samples will be transmitted after this function call
		}
		if(incomingIdx > transmitIdx+1 ){
			bufferedSamples+=(incomingIdx-(transmitIdx+1))*AUDIO_BLOCK_SAMPLES;
		}
		else if(incomingIdx < transmitIdx){
			bufferedSamples += incomingIdx*AUDIO_BLOCK_SAMPLES;
			if(transmitIdx +1 < ringTxBufferSize){
				bufferedSamples += (ringTxBufferSize-(transmitIdx +1))*AUDIO_BLOCK_SAMPLES;
			}
		}
		return bufferedSamples-target;	//-target because it's assumed that target number of samples will be transmitted after this function call
	}
	
	void updateDevCounter(float bufferDiff, uint32_t& devCounter, int8_t& sign){
		if (bufferDiff > 2){
			if(sign != 1){
				devCounter=1;
				sign=1;
			}
			else {
				devCounter++;
			}
		}
		else if (bufferDiff < -2){
			if(sign != -1){
				devCounter=1;
				sign =-1;
			}
			else {
				devCounter++;
			}
		}
	}

	uint32_t getTransmissionTarget(){
		static uint32_t count=0;
		static uint32_t correction =0;
		//compute how many samples we have to transmit ===============
		//number of samples that should be transmitted after 'count' executions of usb_audio_transmit_callback
		uint32_t expected = (count * samplingRate *audioPollingIntervaluS) / 1000000;
		//number of samples that were actual transmitted after 'count' executions of usb_audio_transmit_callback
		uint32_t actual = count *noSamplesPerPollingInterval+correction;

		uint32_t missingSamples = expected-actual;
		uint32_t target=noSamplesPerPollingInterval;
		if(missingSamples != 0){// TODO: dynamic adjust to match USB rate
			correction++;
			target++;
		}
		bool cycleFinished = (count*samplingRate*audioPollingIntervaluS)%1000000 ==0;
		if(cycleFinished){
			count=0;
			correction =0;
		}
		count++;
		return target;
	}
	
	void resetTransmissionIndex(float virtualSamples, uint16_t incomingIdx, uint16_t& idx, uint16_t& count){
		uint16_t targetNoSamples =uint16_t(targetNumTxBufferedSamples-virtualSamples  + 0.f);
		uint16_t targetNumTxBufferedBlocks = uint16_t((targetNoSamples+noSamplesPerPollingInterval)/AUDIO_BLOCK_SAMPLES);	//+noSamplesPerPollingInterval because we will immediatelly transmit 'noSamplesPerPollingInterval' samples
		count = AUDIO_BLOCK_SAMPLES-(uint16_t(targetNoSamples+noSamplesPerPollingInterval)-targetNumTxBufferedBlocks*AUDIO_BLOCK_SAMPLES);
		idx = (incomingIdx -(targetNumTxBufferedBlocks+1)+ringTxBufferSize)%ringTxBufferSize;
	}
}
bool AudioOutputUSB::update_responsibility;
double AudioOutputUSB::updateCurrentSmooth=-1.;
uint16_t AudioOutputUSB::outgoing_count;

/*DMAMEM*/ uint8_t usb_audio_transmit_buffer[AUDIO_TX_SIZE] __attribute__ ((used, aligned(32)));

static void tx_event(transfer_t *t)
{
	int len = usb_audio_transmit_callback();
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&tx_transfer, usb_audio_transmit_buffer, len, 0);
	arm_dcache_flush_delete(usb_audio_transmit_buffer, len);
	usb_transmit(AUDIO_TX_ENDPOINT, &tx_transfer);
}

void AudioOutputUSB::releaseBlocks(uint16_t bufferIdx){
	for (uint16_t j =0; j< noTransmittedChannels; j++){
		if(txBuffer[bufferIdx][j]){
			AudioStream::release(txBuffer[bufferIdx][j]);
			txBuffer[bufferIdx][j]=NULL;
		}
	}	
}

void AudioOutputUSB::begin(void)
{
	outgoing_count = 0;
	for (uint16_t i =0; i< ringTxBufferSize; i++){
		for (uint16_t j =0; j< USB_AUDIO_MAX_NO_CHANNELS; j++){
			txBuffer[i][j]=NULL;
		}
	}
	_lastCallUpdate.reset(blockDuration*F_CPU_ACTUAL);
}

#if AUDIO_SUBSLOT_SIZE==2
static void copy_from_buffers(uint8_t *dst, audio_block_t *blocks[USB_AUDIO_MAX_NO_CHANNELS], unsigned int count, unsigned int len) {
	uint16_t* dst16Bit = (uint16_t*)dst;
	for (uint32_t i =0; i< len; i++){
		for (uint16_t j =0; j< noTransmittedChannels; j++){
			*dst16Bit++ =blocks[j]->data[count +i];
		}
	}
}
#endif

#if AUDIO_SUBSLOT_SIZE==3
static void copy_from_buffers(uint8_t *dst, audio_block_t *blocks[USB_AUDIO_MAX_NO_CHANNELS], unsigned int count, unsigned int len) {
	for (uint32_t i =0; i< len; i++){
		for (uint16_t j =0; j< noTransmittedChannels; j++){
			*dst++ =0;
			*dst++ =((blocks[j]->data[count +i])) & 255;
			*dst++ =((blocks[j]->data[count +i]) >> 8) & 255;
		}
	}
}
#endif

AudioOutputUSB::Status AudioOutputUSB::getStatus() const{
	AudioOutputUSB::Status status;
	NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
	status.usb_audio_underrun_count = txUsb_audio_underrun_count;
	status.usb_audio_overrun_count = txUsb_audio_overrun_count;
	status.target_num_buffered_samples = targetNumTxBufferedSamples;
	status.num_transmitted_channels = noTransmittedChannels;
	status.ring_buffer_size = ringTxBufferSize;
	status.usb_rx_tx_buffer_size = AUDIO_TX_SIZE;
	status.transmittingData=_streaming;
	status.bInterval_uS = audioPollingIntervaluS;
	status.num_skipped_Samples = num_skipped_Samples;
	status.num_padded_Samples = num_padded_Samples;
	NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	return status;
}

void AudioOutputUSB::update(void)
{	
	//update time measurement of update calls
	uint32_t t = ARM_DWT_CYCCNT;
	_lastCallUpdate.addCall(t);
	History<50> historyUpdate = _lastCallUpdate.getHistory();	
	double updateCurrentSmoothL= _lastCallUpdate.getLastCall<20>(historyUpdate, blockDuration*F_CPU_ACTUAL);
	//=======================================
	
	__disable_irq();
		_streaming=transmit_flag != 0;		
		transmit_flag =0;
		if(txBufferState < overrun && incoming_tx_bIdx == transmit_tx_bIdx){
			txBufferState=overrun;
		}
		if(txBufferState == overrun){	
			//only set in case of buffer overrun. Normally updateCurrentSmooth should only be set in combination with the incoming_tx_bIdx incrementation
			updateCurrentSmooth= updateCurrentSmoothL;
		}
		BufferState s = txBufferState;
	__enable_irq();
	if(!_streaming){
		num_skipped_Samples=0;
		num_padded_Samples=0;
		txUsb_audio_underrun_count=0;
		txUsb_audio_overrun_count=0;
		return;
	}
	if(s == overrun){
		txUsb_audio_overrun_count++;
		return;
	}

	for (uint16_t i =0; i< noTransmittedChannels; i++){
		if(txBuffer[incoming_tx_bIdx][i]){
			release(txBuffer[incoming_tx_bIdx][i]);
		}
		txBuffer[incoming_tx_bIdx][i]=receiveReadOnly(i);
		if(!txBuffer[incoming_tx_bIdx][i]){
			if(!txBuffer[incoming_tx_bIdx][i]){
				txBuffer[incoming_tx_bIdx][i] = AudioStream::allocate();
			}
			if(txBuffer[incoming_tx_bIdx][i]){
				memset(txBuffer[incoming_tx_bIdx][i]->data, 0, AUDIO_BLOCK_SAMPLES*sizeof(txBuffer[incoming_tx_bIdx][i]->data[0]));
			}
			else {
				//we ran out of audio memory
				AudioOutputUSB::releaseBlocks(incoming_tx_bIdx);
				break;
			}
		}
	}
	__disable_irq();
		updateCurrentSmooth= updateCurrentSmoothL;
		incoming_tx_bIdx=(incoming_tx_bIdx+1)%ringTxBufferSize;
		if(txBufferState < full && incoming_tx_bIdx == transmit_tx_bIdx){
			txBufferState = full;
		}
	__enable_irq();
}

void AudioOutputUSB::tryIncreaseIdxTransmission(uint16_t& tBIdx, uint16_t& offset){
	AudioOutputUSB::releaseBlocks(tBIdx);			
	uint16_t idxCandidate =(tBIdx+1)%ringTxBufferSize;
	if(idxCandidate != incoming_tx_bIdx){
		tBIdx =(tBIdx+1)%ringTxBufferSize;
		offset=0;
	}
}
float AudioOutputUSB::getBufferedSamples() const{
	NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
	float b = bufferedTxSamples;
	NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	return b;
}
float AudioOutputUSB::getBufferedSamplesSmooth() const{
	NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
	float b = bufferedTxSamplesSmooth;
	NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	return b;
}

// Called from the USB interrupt when ready to transmit another
// isochronous packet.  If we place data into the transmit buffer,
// the return is the number of bytes.  Otherwise, return 0 means
// no data to transmit
unsigned int usb_audio_transmit_callback(void)
{
	uint32_t current =ARM_DWT_CYCCNT;
	lastCallTransmitIsr.addCall(current);
	transmit_flag =1;
	//compute the number of samples we want to transmit (at 44.1kHz that is either 44 or 45 samples)
	uint32_t target = getTransmissionTarget();
	
	const uint16_t iBIdx = incoming_tx_bIdx;	//we are not allowed to change incoming_tx_bIdx 
	uint16_t tBIdx = transmit_tx_bIdx;
	uint16_t offset = AudioOutputUSB::outgoing_count;
	//============================================================
	static uint32_t devCounter=0;
	static int8_t sign =0;
	virtualSamples =0.f;
	if(AudioOutputUSB::updateCurrentSmooth !=-1.){
		
		History<7> historyIsr = lastCallTransmitIsr.getHistory();
		float lastIsrSmooth = (float)lastCallTransmitIsr.getLastCall<2>(historyIsr, expectedIsrIntervalCycles);
		float timeSinceLastUpdate = (float)toUInt32Range(lastIsrSmooth - AudioOutputUSB::updateCurrentSmooth);
		timeSinceLastUpdate /= F_CPU_ACTUAL; //to seconds
		if (timeSinceLastUpdate > 1.5f*blockDuration || timeSinceLastUpdate < -0.5f*blockDuration){
			timeSinceLastUpdate =0.5f*blockDuration;
		}		
		virtualSamples =timeSinceLastUpdate * AUDIO_SAMPLE_RATE;
		
		bufferedTxSamples= getNumBufferedTxSamples(txBufferState, target, iBIdx, tBIdx, offset);
		bufferedTxSamplesSmooth=bufferedTxSamples +  virtualSamples;
		
		// if(Serial){
		// 	Serial.print(bufferedTxSamples);
		// 	Serial.print(" ");
		// 	Serial.print(bufferedTxSamplesSmooth);
		// 	Serial.print(" ");
		// 	Serial.print(bufferedTxSamplesSmooth -targetNumTxBufferedSamples);
		// 	Serial.print(" ");
		// 	Serial.print(target);
		// 	Serial.print(" ");
		// 	Serial.println(txBufferState*100);
		// }
		updateDevCounter(bufferedTxSamplesSmooth -targetNumTxBufferedSamples, devCounter, sign);		
	}
	
	if(txBufferState == AudioOutputUSB::overrun){
		devCounter=0;
		resetTransmissionIndex(virtualSamples, iBIdx, tBIdx, offset);		
		for (uint16_t idx =0; idx < ringTxBufferSize; idx++){		
			AudioOutputUSB::releaseBlocks(idx);
		}
		txBufferState=AudioOutputUSB::ready;
	}
	
	
	uint32_t len=0;
	uint8_t *data = usb_audio_transmit_buffer;
	while (len < target) {
		uint32_t num = target - len;
		uint32_t avail = AUDIO_BLOCK_SAMPLES - offset;
		if (num > avail){
			num = avail;
		}
		if( avail==0 ||	txBuffer[tBIdx][0]==NULL){
			//Something went wrong. We either did not receive a block, or a buffer underrun occured.
			//We will reset the buffer indices and offsets and transmit zeros.
			if( avail==0){
				devCounter=0;	//only reset in case of an underrun
				txUsb_audio_underrun_count++;
				resetTransmissionIndex(virtualSamples, iBIdx, tBIdx, offset);		
				for (uint16_t idx =0; idx < ringTxBufferSize; idx++){
					AudioOutputUSB::releaseBlocks(idx);
				}
			}
			const uint32_t numBytes = num*noTransmittedChannels*AUDIO_SUBSLOT_SIZE;
			memset(data, 0, numBytes);
		}
		else {
			copy_from_buffers(data, txBuffer[tBIdx], offset, num);
		}
		data += num*noTransmittedChannels*AUDIO_SUBSLOT_SIZE;
		len+=num;
		offset+=num;
		if(devCounter == 10){
			if(sign == -1 && offset > 0){
				devCounter=0;
				num_padded_Samples++;
				offset--;
			}
		}
		if (offset >= AUDIO_BLOCK_SAMPLES) {
			AudioOutputUSB::tryIncreaseIdxTransmission(tBIdx,offset);
		}
	}

	if(devCounter == 10 && sign ==1 && offset < AUDIO_BLOCK_SAMPLES){
		devCounter=0;
		num_skipped_Samples++;
		//we remove one sample from the buffer
		offset++;
		if (offset == AUDIO_BLOCK_SAMPLES) {
			AudioOutputUSB::tryIncreaseIdxTransmission(tBIdx,offset);
		}
	}
	transmit_tx_bIdx=tBIdx;
	AudioOutputUSB::outgoing_count = offset;
	return target * noTransmittedChannels*AUDIO_SUBSLOT_SIZE;
}
#endif





void usb_audio_configure(void)
{
	rxUsb_audio_underrun_count = 0;
	rxUsb_audio_overrun_count = 0;
	rxMemoryUnderrunCounter=0;
	sumDiff=0.;
	lastDiff=0.;
	rxBufferReady= false;
	incoming_rx_bIdx=0;
	transmit_rx_bIdx=0;
	if (usb_high_speed) {
		noTransmittedChannels = USB_AUDIO_NO_CHANNELS_480;
		audioPollingIntervalSec = AUDIO_POLLING_INTERVAL_480_SEC;
		audioPollingIntervaluS = AUDIO_NUM_SUBFRAMES_PER_POLLING_480 *125;
		usb_audio_sync_nbytes = 4;
		usb_audio_sync_rshift = 8;
	} else {
		noTransmittedChannels = USB_AUDIO_NO_CHANNELS_12;
		audioPollingIntervalSec = AUDIO_POLLING_INTERVAL_12_SEC;
		audioPollingIntervaluS = AUDIO_NUM_SUBFRAMES_PER_POLLING_12*125;
		usb_audio_sync_nbytes = 3;
		usb_audio_sync_rshift = 10;
	}

	noSamplesPerPollingInterval = (samplingRate*audioPollingIntervaluS)/1000000;

	feedback_accumulator_default = uint32_t((samplingRate *audioPollingIntervalSec) * 0x1000000 +0.5f);
	feedback_accumulator = feedback_accumulator_default;

	memset(&rx_transfer, 0, sizeof(rx_transfer));
	usb_config_rx_iso(AUDIO_RX_ENDPOINT, AUDIO_RX_SIZE, 1, rx_event);
	rx_event(NULL);
	memset(&sync_transfer, 0, sizeof(sync_transfer));
	usb_config_tx_iso(AUDIO_SYNC_ENDPOINT, usb_audio_sync_nbytes, 1, sync_event);
	sync_event(NULL);
	memset(&tx_transfer, 0, sizeof(tx_transfer));
	usb_config_tx_iso(AUDIO_TX_ENDPOINT, AUDIO_TX_SIZE, 1, tx_event);
	tx_event(NULL);
	expectedIsrIntervalCycles = audioPollingIntervalSec *F_CPU_ACTUAL;
	lastCallReceiveIsr.reset(expectedIsrIntervalCycles);

	//AudioOutputUSB	==============================
	lastCallTransmitIsr.reset(expectedIsrIntervalCycles);		
	num_skipped_Samples=0;
	num_padded_Samples=0;
	txUsb_audio_underrun_count=0;
	txUsb_audio_overrun_count=0;
	//=================================================
}






struct setup_struct {
  //union {
    struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	union {
		struct {
			uint8_t bChannel;  // 0=main, 1=left, 2=right
			uint8_t bCS;       // Control Selector
		};
		uint16_t wValue;
	};
	union {
		struct {
			uint8_t bIfEp;     // type of entity
			uint8_t bEntityId; // UnitID, TerminalID, etc.
		};
		uint16_t wIndex;
	};
	uint16_t wLength;
    };
  //};
};

int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen)
{

	struct setup_struct setup = *((struct setup_struct *)stp);
	if(setup.bmRequestType != 0xA1){
		//was not a get feature request directed to the feature unit
		return 0;
	}
	
	if (setup.bRequest==0x02) { // -> request = RANGE
		if (setup.bCS==0x01) { // mute
			data[0] =1; //only one sub-range (LSB of 2 bytes)
			data[1] =0; //only one sub-range (MSB of 2 bytes)
			data[2] =0;	//unmute
			data[3] = 1; //mute
			data[4] = 1; //resolution
			*datalen = 5;
			return 1;
		}
		else if (setup.bCS==0x02) { // volume
			data[0] = 1; //only one sub-range (LSB of 2 bytes)
			data[1] = 0; //only one sub-range (MSB of 2 bytes)
			data[2] = 0;	// min level is 0 (LSB)
			data[3] = 0; 	// min level is 0 (MSB)
			data[4] = FEATURE_MAX_VOLUME;  	// max level, for range of 0 to MAX (LSB)
			data[5] = 0;					// max level, for range of 0 to MAX (MSB)
			data[6] = 1; // increment vol by by 1 (LSB)
			data[7] = 0; // increment vol by by 1 (MSB)
			*datalen = 8;
			return 1;
		}
		else { // pass over SET_MEM, etc.
			return 0;
		}
	}
	if (setup.bRequest==0x01) { // -> request = CUR
		if (setup.bCS==0x01) { // mute
			data[0] = AudioInputUSB::features.mute;  // 1=mute, 0=unmute
			*datalen = 1;
			return 1;
		}
		else if (setup.bCS==0x02) { // volume
			data[0] = AudioInputUSB::features.volume & 0xFF;	//(LSB)
			data[1] = (AudioInputUSB::features.volume>>8) & 0xFF; //(MSB)
			*datalen = 2;
			return 1;
		}
		else { // pass over SET_MEM, etc.
			return 0;
		}
	}
	return 0;
}

int usb_audio_set_feature(void *stp, uint8_t *buf) 
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0x21) { // SET FEATURE should check bRequest, bChannel and UnitID
			if (setup.bCS==0x01) { // mute
				if (setup.bRequest==0x01) { // CUR
					// if(Serial){
					// 	Serial.println("set mute");
					// }
					AudioInputUSB::features.mute = buf[0]; // 1=mute,0=unmute
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x01) { // CUR
					// if(Serial){
					// 	Serial.println("set volume");
					// }
					AudioInputUSB::features.volume = buf[0];
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
	}
	return 0;
}


#endif // AUDIO_INTERFACE
