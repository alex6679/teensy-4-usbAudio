/*
 * Testbed sketch for multi-channel USB audio
 */
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

// GUItool: begin automatically generated code
AudioSynthWaveform       wav1; //xy=479,580
AudioSynthWaveform       wav2;      //xy=484,616
AudioSynthWaveform       wav3; //xy=486,654
AudioSynthWaveform       wav4; //xy=490,691
AudioSynthWaveform       wav5; //xy=494,729
AudioSynthWaveform       wav6; //xy=499,765
AudioSynthWaveform       wav7; //xy=503,802
AudioSynthWaveform       wav8; //xy=507,840
AudioInputUSBOct         usb_oct_in;       //xy=524,931
AudioMixer4              mixer1;         //xy=728,896
AudioMixer4              mixer2; //xy=733,968
AudioMixer4              mixer3; //xy=870,946
AudioOutputI2S           i2sOut;           //xy=939,684
AudioOutputUSBOct        usb_oct_out;       //xy=942,589

AudioConnection          patchCord1(wav1, 0, usb_oct_out, 0);
AudioConnection          patchCord2(wav1, 0, i2sOut, 0);
AudioConnection          patchCord3(wav2, 0, usb_oct_out, 1);
AudioConnection          patchCord4(wav3, 0, usb_oct_out, 2);
AudioConnection          patchCord5(wav4, 0, usb_oct_out, 3);
AudioConnection          patchCord6(wav5, 0, usb_oct_out, 4);
AudioConnection          patchCord7(wav6, 0, usb_oct_out, 5);
AudioConnection          patchCord8(wav7, 0, usb_oct_out, 6);
AudioConnection          patchCord9(wav8, 0, usb_oct_out, 7);
AudioConnection          patchCord10(usb_oct_in, 0, mixer1, 0);
AudioConnection          patchCord11(usb_oct_in, 1, mixer1, 1);
AudioConnection          patchCord12(usb_oct_in, 2, mixer1, 2);
AudioConnection          patchCord13(usb_oct_in, 3, mixer1, 3);
AudioConnection          patchCord14(usb_oct_in, 4, mixer2, 0);
AudioConnection          patchCord15(usb_oct_in, 5, mixer2, 1);
AudioConnection          patchCord16(usb_oct_in, 6, mixer2, 2);
AudioConnection          patchCord17(usb_oct_in, 7, mixer2, 3);
AudioConnection          patchCord18(mixer1, 0, mixer3, 0);
AudioConnection          patchCord19(mixer2, 0, mixer3, 1);
AudioConnection          patchCord20(mixer3, 0, i2sOut, 1);

AudioControlSGTL5000     sgtl5000;     //xy=946,745
// GUItool: end automatically generated code



AudioSynthWaveform* wavs[] = {
  &wav1,
  &wav2,
  &wav3,
  &wav4,
  &wav5,
  &wav6,
  &wav7,
  &wav8
};

AudioMixer4* mixers[] = {&mixer1,&mixer2};

uint32_t ledOff;

void setup() 
{
  pinMode(LED_BUILTIN,OUTPUT);

  // OK with 128 or 256 sample blocks, not working with 16 samples (yet)
  AudioMemory(150 * 128 / AUDIO_BLOCK_SAMPLES); // empirical calculation!

  while (!Serial)
    ;

  if (CrashReport)
    Serial.print(CrashReport);
  
  sgtl5000.setAddress(HIGH);
  sgtl5000.enable();
  sgtl5000.volume(0.05f);

  for (int i=0;i<8;i++)
  {
    wavs[i]->begin(0.5f,220.0f + 110.0f*i,WAVEFORM_TRIANGLE);
    wavs[i]->phase(15.0f*i);
  }

  for (int i=0;i<2;i++)
  {
    for (int j=0;j<4;j++)
      mixers[i]->gain(j,0.25f);
  }

  Serial.printf("Audio block size %d samples; sample rate %.2f; %d channels\n",AUDIO_BLOCK_SAMPLES,AUDIO_SAMPLE_RATE_EXACT,AUDIO_CHANNELS);
  Serial.println("Running");
}

uint32_t lastBlocks;
void loop() 
{
  if (millis() > ledOff)
  {
    digitalWrite(LED_BUILTIN,0);  
  }

  if (millis() - lastBlocks > 500)
  {
    lastBlocks = millis();
    Serial.printf("Blocks %d; max %d\n",AudioMemoryUsage(),AudioMemoryUsageMax());
    AudioMemoryUsageMaxReset();
  }
}
