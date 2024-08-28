# Extension of the T4.x USB audio input and output. #
This suite of files enables 2-, 4-, 6- or 8-channel USB I/O to be used with the Teensy 4.x Audio library. It has been devloped based on Teensyduino 1.59, so no representations are made as to whether it might work starting from older versions!

## Installation ##
Until such time as these changes are merged into the Teensyduino release package, installation is a manual process. These instructions assume you are working from an unmodified installation - if you have already modified any of the files involved you will need to figure out how to merge these changes yourself.

**It is strongly recommended that you keep safety copies of any files that you overwrite or delete, so that if something goes wrong you can go back to a working installation.**

### Core files ###
These provide the multi-channel USB audio capability.
* Locate the core files folder, e.g. for Arduino 1.8.19 on Windows `C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy4`
* Copy the contents of the `changedCorefiles` folder into the cores folder, overwriting as necessary

### GUI files ###
These provide the Design Tool with ability to create an audio design using multi-channel USB objects. 
* Locate the audio library GUI folder, e.g. for Arduino 1.8.19 on Windows `C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Audio\gui`
* Copy the contents of the `changedGUI` folder into the GUI folder, overwriting as necessary

### Config files ###
These provide the Arduino IDE with extra entries in the Tools menu, allowing selection of the USB channel count. 
* Locate the config files folder, e.g. for Arduino 1.8.19 on Windows `C:\Program Files (x86)\Arduino\hardware\teensy\avr`
* If you have no `boards.local.txt`
  * simply copy this in from the `changedConfigfiles` folder
* else 
  * merge the contents of `changedConfigfiles/boards.local.txt` into your existing file
* Make a safety copy of `platform.txt`, then delete the one in the config folder
* If you are using Arduino IDE 1.x (e.g. 1.8.19)
  * copy `changedConfigfiles/TD_platform.txt` into the config folder, and rename to `platform.txt`
* else (you are using Arduino IDE 2.x)
  * copy `changedConfigfiles/BM_platform.txt` into the config folder, and rename to `platform.txt`
  * ensure the IDE is *not* running
  * find the Arduino 2.x cache folder, e.g. `C:\Users\<user name>\AppData\Roaming\arduino-ide\` and delete it

## In use ##
* In the updated Design Tool, place one usb, usb_quad, usb_hex or usb_oct input and/or output object(s) in your design, and export as usual
* In the Tools menu, ensure you have configured the `USB Type` to include Audio, and select the correct number of `USB channels`
* If your sketch uses a USB I/O object that is wider than configured, you will get a compile-time error - usually *many* errors
* Windows is very bad at detecting changes to the USB descriptor: see the examples for how to use the `set_usb_string_serial_number.h` file to change the serial number according to the channel count and sample rate, which seems to force Windows to recognise the changes
* In addition to the USB channel count, the Tools menu also has options to
  * set the sample rate to 44.1kHz, 48kHz or 96kHz: these seem to work OK, but may not be supported by all audio hardware
  * set the audio block sample count to 128 (normal), 16, or 256 samples. This is highly experimental, and many audio objects work badly if the sample count is changed. It is hoped that future revisions of the Audio library will be more resilient to changing this parameter, which will be of use for (a) low-latency applications using 16-sample blocks, or (b) keeping the audio interrupt rate reasonable at 96kHz  sample rate by using 256-sample blocks

## Examples ##  
Examples can be found in `main_usbOutput.cpp` and `main_usbInput.cpp` 

## Technical details ## 
Main features are:
- Switched from UAC1 to UAC2 standard.
- 8 channels can be streamed from and to the USB host. (Change `USB_AUDIO_NO_CHANNELS_480` in `usb_desc.h` if you can't get the updated Tools options to work. For more than 8 channels one need to extend `CHANNEL_CONFIG_480` and the feature unit descriptor in `usb_desc.c`)
- 16 or 24bit pcm audio can be streamed. (Change `AUDIO_SUBSLOT_SIZE` in `usb_desc.h` if needed. `AUDIO_SUBSLOT_SIZE 2` means each sample is 2bytes large, `AUDIO_SUBSLOT_SIZE 3` changes the sample size to 3bytes/24bit)
- Feedback of the usb input to the host improved in order to prevent buffer under- and overruns
- USB output is able to duplicate or skip single samples in order to prevent buffer under- and overruns. This is not a perfect solution, but is an improvement to the current implementation.
- USB input: Parameters of the PI controller that computes the feedback can optionally be set at the constructor.
- USB input and output: The target number of buffered samples can be configured. (Can e.g. be increased if buffer under- or overruns occur.)
- USB input and ouput provide information about their status (getStatus) like if and how many buffer over- and under-runs occurred.
