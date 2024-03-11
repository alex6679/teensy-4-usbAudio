Extension of the T4 and T4.1 USB audio input and output.
Main features are:
- Switched from UAC1 to UAC2 standard.
- 8 channels can be streamed from and to the USB host. (Change USB_AUDIO_NO_CHANNELS_480 in usb_desc.h if needed. For more than 8 channels one need to extend CHANNEL_CONFIG_480 and the feature unit descriptor in usb_desc.c)
- 16 or 24bit pcm audio can be streamed. (Change AUDIO_SUBSLOT_SIZE in usb_desc.h if needed. AUDIO_SUBSLOT_SIZE 2 means each sample is 2bytes large, AUDIO_SUBSLOT_SIZE 3 changes the sample size to 3bytes/24bit)
- Feedback of the usb input to the host improved in order to prevent buffer under- and overruns
- USB output is able to duplicate or skip single samples in order to prevent buffer under- and overruns. This is not a perfect solution, but is an improvement to the current implementation.
- USB input: Parameters of the PI controller that computes the feedback can optionally be set at the constructor.
- USB input and output: The target number of buffered samples can be configured. (Can e.g. be increased if buffer under- or overruns occur.)
- USB input and ouput provide information about there status (getStatus) like if and how many buffer over- and underruns occured.

Tested with Teensyduino 1.59
In order to use it, copy the files of 'changedCorefiles' into cores/Teensy4 and replace the original files.
Examples can be found in main_usbOutput.cpp and main_usbInput.cpp 
