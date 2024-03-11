
#include <Arduino.h>
#include "plotter.h"

void Plotter::activate(bool on){
    _on=on;
}

void Plotter::update(void) {
    const uint8_t noChannels=8;
    audio_block_t *blocks[noChannels];
    int16_t *dataPtr[noChannels];
    for (uint8_t i =0; i< noChannels; i++) {
        blocks[i] = receiveWritable(i);
        if(blocks[i]){
            dataPtr[i]=blocks[i]->data;
        }
        else{
            dataPtr[i]=nullptr;
        }
    }
    if(_on/* && counter <6 */) {
        // Serial.print("counter: ");
        // Serial.println(counter);
        // counter++;
        for (uint16_t i =0; i< AUDIO_BLOCK_SAMPLES; i++){
            for (uint8_t j =0; j< noChannels; j++){
                if(dataPtr[j]){
                    if(i%_step==0){
                        Serial.print(*dataPtr[j]);                  
                        Serial.print(" ");
                    }
                    ++dataPtr[j];
                }
            }
            if(i%_step==0){
                Serial.println();
            }
        }
    }
	for (uint8_t i =0; i< noChannels; i++) {
        if(blocks[i]) {
            release(blocks[i]);
            blocks[i]=nullptr;
        }
    }
}


