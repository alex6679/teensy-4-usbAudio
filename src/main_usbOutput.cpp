#include <Audio.h>
#include "set_usb_string_serial_number.h"

//activate one of the two options
//#define PLOT_BUFFER             //use Arduino Serial Plotter
#define PRINT_USBOUTPUT_STATUS   //prints information like number of buffer over and underruns

AudioSynthWaveform       audioSynth0;
AudioSynthWaveform       audioSynth1;
AudioSynthWaveform       audioSynth2;
AudioSynthWaveform       audioSynth3;
AudioSynthWaveform       audioSynth4;
AudioSynthWaveform       audioSynth5;
AudioSynthWaveform       audioSynth6;
AudioSynthWaveform       audioSynth7;
AudioOutputUSB           usb1;           
AudioOutputI2S           i2s1;          
AudioConnection          patchCord1(audioSynth0, 0, usb1, 0);
AudioConnection          patchCord2(audioSynth1, 0, usb1, 1);
AudioConnection          patchCord3(audioSynth2, 0, usb1, 2);
AudioConnection          patchCord4(audioSynth3, 0, usb1, 3);
AudioConnection          patchCord5(audioSynth0, 0, usb1, 4);
AudioConnection          patchCord6(audioSynth1, 0, usb1, 5);
AudioConnection          patchCord7(audioSynth2, 0, usb1, 6);
AudioConnection          patchCord8(audioSynth3, 0, usb1, 7);
AudioSynthWaveform* wavs[] = {
  &audioSynth0,
  &audioSynth1,
  &audioSynth2,
  &audioSynth3,
  &audioSynth4,
  &audioSynth5,
  &audioSynth6,
  &audioSynth7
};

void setup() {                
  AudioMemory(80);
  for (int i=0;i<8;i++) {
    wavs[i]->begin(0.5f,220.0f + 110.0f*i,WAVEFORM_TRIANGLE);
    wavs[i]->phase(15.0f*i);
  }
  Serial.begin(115200);
  while (!Serial){};
  // delay(5000);
  // printRequestBuffer(); // only used for debugging the usb requests from the host at the initialization
}

void loop() {

#ifdef PLOT_BUFFER
    Serial.print(usb1.getBufferedSamples());
    Serial.print(" ");
    Serial.println(usb1.getBufferedSamplesSmooth());
    delay(200); 
#endif
 
#ifdef PRINT_USBOUTPUT_STATUS
    USBAudioOutInterface::Status status = usb1.getStatus();
    Serial.print("buffer overrun: ");
    Serial.println(status.usb_audio_overrun_count);
    Serial.print("buffer underruns: ");
    Serial.println(status.usb_audio_underrun_count);
    Serial.print("buffer target number of samples : ");
    Serial.println(status.target_num_buffered_samples);
    Serial.print("number of transmitted channels : ");
    Serial.println(status.num_transmitted_channels);
    Serial.print("size of ring buffer : ");
    Serial.println(status.ring_buffer_size);  
    Serial.print("currently transmitting data : ");
    Serial.println(status.transmittingData);  
    Serial.print("num_padded_Samples : ");
    Serial.println(status.num_padded_Samples);
    Serial.print("num_skipped_Samples : ");
    Serial.println(status.num_skipped_Samples);
    Serial.print("max memory: ");
    Serial.println((AudioMemoryUsageMax()));
    delay(1000);
#endif
}
