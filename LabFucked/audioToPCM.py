# audioToPCM.py
# By: Alex Anderson-McLeod
# Converts an 8-bit .wav file to a C array of bytes, saved in a file called array.h.
# Sample rate is also saved in the file.
# Array is called sampleArray and sample rate is called sampleRate.
# The program will automatically cut off the array once the ESP32 runs out of memory.
# Usage: python3 audioToPCM.py <mySong.wav> <desiredSampleRate>

import wave
import sys
import numpy as np
from scipy.signal import resample

flash_size = 950000  # ~1 MB of flash memory for audio storage

try:
    filename = sys.argv[1]
    if len(sys.argv) < 3:
        auto_sample_rate = True
    else:
        desired_rate = abs(int(sys.argv[2]))
        auto_sample_rate = False
except:
    print(f"usage: python3 {sys.argv[0]} <mySong.wav> <desiredSampleRate>")
    sys.exit()

try:
    with wave.open(filename) as wav_file:
        metadata = wav_file.getparams()

        num_channels = metadata.nchannels
        sample_width = metadata.sampwidth
        sample_rate = metadata.framerate

        if auto_sample_rate:
            # Auto-pick sample rate to fit memory
            desired_rate = int(flash_size * sample_rate / metadata.nframes)
            if desired_rate > sample_rate:
                desired_rate = sample_rate

            print(f"No sample rate specified, auto-picking {desired_rate} Hz to fit the entirety of the audio into memory.")

            if desired_rate < 8000:
                print("Warning: <8000 Hz may sound very low quality.")

        print("Converting input file with the following parameters:")
        print(f"Channels: {num_channels}")
        print(f"Sample Width: {sample_width * 8} Bits")
        print(f"Sample Rate: {sample_rate} Hz")
        print(f"Number of Frames: {metadata.nframes}")

        print("To an output array.h with the following parameters:")
        print("Channels: 1")
        print("Sample Width: 8 Bits")
        print(f"Sample Rate: {desired_rate} Hz")

        esp32_samples = int(metadata.nframes * desired_rate / sample_rate)

        # FIXED SECTION
        if esp32_samples < flash_size:
            frame_info = esp32_samples
        else:
            frame_info = f"{flash_size} (Audio truncated to fit ESP32 memory)"

        print(f"Number of Frames: {frame_info}")

        play_time = (esp32_samples if esp32_samples < flash_size else flash_size) / desired_rate
        percent_used = 100 * (flash_size / esp32_samples)

        print(f"The ESP32 will be able to play {play_time:.2f} seconds AKA {percent_used:.2f}% of your audio.")

        frames = wav_file.readframes(metadata.nframes)

        width_map = {
            1: np.uint8,
            2: np.int16,
            3: np.int32,
            4: np.int32
        }

        if sample_width not in width_map:
            print(f"Error: Invalid sample width of {sample_width * 8} bits!")
            sys.exit()

        audio_data = np.frombuffer(frames, dtype=width_map[sample_width])

        # Convert to mono if needed
        if num_channels > 1:
            audio_data = audio_data.reshape(-1, num_channels)
            audio_data = audio_data.mean(axis=1)

        # Normalize to [-1, 1]
        if sample_width == 1:
            audio_data = audio_data.astype(np.float32)
            audio_data = (audio_data - 128) / 127.5
        else:
            audio_data = audio_data.astype(np.float32) / float(2 ** (8 * sample_width - 1))

        # Resample
        num_samples_target = int(len(audio_data) * desired_rate / sample_rate)
        audio_resampled = resample(audio_data, num_samples_target)

        # Clip and convert to 8-bit unsigned
        audio_resampled = np.clip(audio_resampled, -1.0, 1.0)
        out_data = ((audio_resampled + 1.0) * 127.5).astype(np.uint8)

        # Write to header file
        with open("array.h", "w") as out_file:
            out_file.write("const uint32_t sampleRate = ")
            out_file.write(str(desired_rate))
            out_file.write(";\n")

            out_file.write("const uint8_t sampleArray[] = {")

            for index, element in enumerate(out_data):
                out_file.write(str(element))
                if index != len(out_data) - 1 and index < flash_size:
                    out_file.write(", ")
                else:
                    break

            out_file.write("};")

except:
    print(f"{filename} is not a valid wave file!")
