# audioToPCM.py
# By: Alex Anderson-McLeod
# Converts an 8-bit .wav file to a C array of bytes, saved in a file called array.h. Sample rate is also saved in the file.
# Array is called sampleArray and sample rate is called sampleRate.
# The program will automatically cut off the array once the ESP32 runs out of memory.
# Usage: python audioToPCM.py <mySong.wav>

import wave
import sys

try:
    filename = sys.argv[1]
except:
    print(f"usage: python {sys.argv[0]} <mySong.wav>")
    sys.exit()
try:
    with wave.open(filename) as wav_file:
        metadata = wav_file.getparams()
        print(metadata)
        frames = wav_file.readframes(metadata.nframes)
        sample_array = [x for x in frames]
        with open("code/array.h", "w") as out_file:
            out_file.write("const uint32_t sampleRate = ")
            out_file.write(str(metadata.framerate))
            out_file.write(";\n")
            out_file.write("const uint8_t sampleArray[] = {")
            for index, element in enumerate(sample_array):
                out_file.write(str(element))
                if index != len(sample_array) - 1 and index < 1000000:
                    out_file.write(", ")
                else: 
                    break
            out_file.write("};")
except:
    print(f"{filename} is not a valid wave file!")