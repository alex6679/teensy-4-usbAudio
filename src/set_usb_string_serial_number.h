#ifndef dec_usb_string_serial_number_h_
#define dec_usb_string_serial_number_h_

#include <AudioStream.h>

#define AUDIO_kHz ((int) AUDIO_SAMPLE_RATE / 1000)
#define AUDIO_CHANNELS USB_AUDIO_NO_CHANNELS_480

// Changing string in descriptor keeps Windows 10 happier
extern "C"
{
    struct usb_string_descriptor_struct
    {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint16_t wString[6+1+1+2+1+1];
    };

  usb_string_descriptor_struct usb_string_serial_number={
    2+(6+1+1+2+1+1)*2,3,
    {'A','u','d','i','o','-','0'+AUDIO_CHANNELS,'/','0'+(AUDIO_kHz / 10),'0' + (AUDIO_kHz % 10),'0'+AUDIO_SUBSLOT_SIZE,'B'}
  };
}

#endif