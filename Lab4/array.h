#ifndef ARRAY_H
#define ARRAY_H

#include <stdint.h>

// Replace this file by running: python audioToPCM.py <yourfile.wav>
const uint32_t sampleRate = 8000;
const uint8_t sampleArray[] = {
    128, 255, 128, 0, 128, 255, 128, 0,
    128, 255, 128, 0, 128, 255, 128, 0
};

#endif