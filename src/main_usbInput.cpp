#include "plotter.h"
#include <Audio.h>
#include "set_usb_string_serial_number.h"

//activate one of the four options
//#define PLOT_SIGNAL             //use Arduino Serial Plotter
//#define PLOT_BUFFER             //use Arduino Serial Plotter
//#define PLOT_REQUEST_FRREQ      //use Arduino Serial Plotter
//#define PLOT_BINTERVAL          //use Arduino Serial Plotter
#define PRINT_VOL_CHANGES         //prints volume changes, use Serial Monitor
//#define PRINT_USBINPUT_STATUS   //prints information like number of buffer over and underruns

AudioInputUSB            usb1;          
AudioOutputI2S           i2s1;        
Plotter                  plotter(4);  //only plot every 4th sample
AudioConnection          patchCordPlotter1(usb1, 0, plotter, 0);
AudioConnection          patchCordPlotter2(usb1, 2, plotter, 1);
uint16_t expectedBIntervalUs;
void setup() {                
  AudioMemory(50);
  Serial.begin(115200);
  while (!Serial){};
#ifdef PLOT_SIGNAL
  plotter.activate(true);
#endif
  // delay(5000);
  // printRequestBuffer();  // only used for debugging the usb requests from the host at the initialization
  USBAudioInInterface::Status status = usb1.getStatus();
  expectedBIntervalUs = status.bInterval_uS;
}

void loop() {
  #ifdef PRINT_VOL_CHANGES
  if (USBAudioInInterface::features.change){
      Serial.println(USBAudioInInterface::features.volume);
      USBAudioInInterface::features.change =0;
  }
  delay(200);
  #endif
#ifdef PLOT_BUFFER
  Serial.print(usb1.getBufferedSamples());
  Serial.print(" ");
  Serial.println(usb1.getBufferedSamplesSmooth());
  delay(200);
#endif
#ifdef PLOT_REQUEST_FRREQ
  Serial.println(usb1.getRequestedSamplingFrequ(),4);
  delay(200);
#endif
#ifdef PLOT_BINTERVAL
  Serial.print(usb1.getActualBIntervalUs(),4);
  Serial.print(" ");
  Serial.println(expectedBIntervalUs);
  delay(200);
#endif
#ifdef PRINT_USBINPUT_STATUS
  USBAudioInInterface::Status status = usb1.getStatus();
  Serial.print("buffer overrun: ");
  Serial.println(status.usb_audio_overrun_count);
  Serial.print("buffer underruns: ");
  Serial.println(status.usb_audio_underrun_count);
  Serial.print("memory underruns: ");
  Serial.println(status.audio_memory_underrun_count);
  Serial.print("buffer target number of samples : ");
  Serial.println(status.target_num_buffered_samples);
  Serial.print("number of transmitted channels : ");
  Serial.println(status.num_transmitted_channels);
  Serial.print("size of ring buffer : ");
  Serial.println(status.ring_buffer_size);  
  Serial.print("size of usb receive and transmit buffers : ");
  Serial.println(status.usb_rx_tx_buffer_size);  
  Serial.print("currently receiving data : ");
  Serial.println(status.receivingData);  
  Serial.print("usb_high_speed : ");
  Serial.println(status.usb_high_speed); 
  Serial.print("bInterval [uS] : ");
  Serial.println(status.bInterval_uS);
  Serial.print("buffered samples (smooth): ");
  Serial.println(usb1.getBufferedSamplesSmooth());
  Serial.print("buffered samples: ");
  Serial.println(usb1.getBufferedSamples());
  Serial.println();
  delay(1000);
#endif
}