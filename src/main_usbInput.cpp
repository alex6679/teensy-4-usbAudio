#include <plotter.h>
#include <Audio.h>

#define AUDIO_kHz ((int) AUDIO_SAMPLE_RATE / 1000)
#define AUDIO_CHANNELS USB_AUDIO_NO_CHANNELS_480

// Changing string in descriptor keeps Windows 10 happier
extern "C"
{
    struct usb_string_descriptor_struct
    {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint16_t wString[6+1+1+2+1];
    };

  usb_string_descriptor_struct usb_string_serial_number={
    2+(6+1+1+2+1)*2,3,
    {'A','u','d','i','o','-','0'+AUDIO_CHANNELS,'/','0'+(AUDIO_kHz / 10),'0' + (AUDIO_kHz % 10),'B'}
  };
}

//#include <util/usbHostRequestBuffer.h>  only used for debugging the initialization

//activate one of the four options
//#define PLOT_SIGNAL               //use Arduino Serial Plotter
#define PLOT_BUFFER             //use Arduino Serial Plotter
//#define PLOT_REQUEST_FRREQ      //use Arduino Serial Plotter
//#define PRINT_USBINPUT_STATUS   //prints information like number of buffer over and underruns

AudioInputUSB            usb1;          
AudioOutputI2S           i2s1;          
Plotter                  plotter(4);  //only plot every 4th sample
AudioConnection          patchCord1(usb1, 0, plotter, 0);
AudioConnection          patchCord2(usb1, 2, plotter, 1);

void setup() {                
  AudioMemory(50);
  Serial.begin(115200);
  while (!Serial){};
#ifdef PLOT_SIGNAL
  plotter.activate(true);
#endif
  // delay(5000);
  // printRequestBuffer();  // only used for debugging the usb requests from the host at the initialization
}

void loop() {
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
#ifdef PRINT_USBINPUT_STATUS
  AudioInputUSB::Status status = usb1.getStatus();
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
  Serial.print("bInterval [uS] : ");
  Serial.println(status.bInterval_uS);
  Serial.print("buffered samples (smooth): ");
  Serial.println(usb1.getBufferedSamplesSmooth());
  Serial.println();
  delay(1000);
#endif
}