/*
The MIT License (MIT)

Copyright (c) 2020 EdgeImpulse Inc.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "MicroBit.h"
#include "ContinuousAudioStreamer.h"
#include "StreamNormalizer.h"
#include "Tests.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

#define INFERENCING_KEYWORD     "microbit"
#define ANOTHER_KEYWORD         "house"

static NRF52ADCChannel *mic = NULL;
static ContinuousAudioStreamer *streamer = NULL;
static StreamNormalizer *processor = NULL;

static inference_t inference;
bool lightIsOn_0 = false; // records if LED at Pin 0 is on or off
bool lightIsOn_1 = false; // records if LED at Pin 1 is on or off
int count = 0; // ensures that the light switching on happens only once per keyword

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int8_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    return 0;
}

/**
 * Invoked when we hear the keyword !
 */
static void heard_keyword() {
    const char * happy_emoji ="\
        000,255,000,255,000\n\
        000,000,000,000,000\n\
        255,000,000,000,255\n\
        000,255,255,255,000\n\
        000,000,000,000,000\n";
    MicroBitImage img(happy_emoji);
    uBit.display.print(img);
    ++count; // count increases when keyword is heard 
    if (lightIsOn_0 && count < 2){ // if light is on and not already turning off
        uBit.io.P0.setDigitalValue(0); // switch off LED at Pin 0
        lightIsOn_0 = false; // LED is now off
    }
    else if (count < 2){ // if light is off and not already turning on
        uBit.io.P0.setDigitalValue(1); // switch on LED at Pin 0
        lightIsOn_0 = true; // LED is now on
    }
}

// Invoked when we hear another keyword
static void heard_another_keyword() {
    const char * empty_emoji ="\
        000,000,255,000,000\n\
        000,255,255,255,000\n\
        255,255,255,255,255\n\
        000,255,255,255,000\n\
        000,255,000,255,000\n";
    MicroBitImage img(empty_emoji);
    uBit.display.print(img);
    ++count; // count increases when keyword is heard 
    if (lightIsOn_1 && count < 2){ // if light is on and not already turning off
        uBit.io.P1.setDigitalValue(0); // switch off LED at Pin 1
        lightIsOn_1 = false; // LED is now off
    }
    else if (count < 2){ // if light is off and not already turning on
        uBit.io.P1.setDigitalValue(1); // switch on LED at Pin 1
        lightIsOn_1 = true; // LED is now on
    }
}

/**
 * Invoked when we hear something else
 */
static void heard_other() {
    const char * empty_emoji ="\
        000,000,000,000,000\n\
        000,000,000,000,000\n\
        000,000,255,000,000\n\
        000,000,000,000,000\n\
        000,000,000,000,000\n";
    MicroBitImage img(empty_emoji);
    uBit.display.print(img);
    count = 0; // resets count
}

 // Invoked when something happens
 static void debug() {
     const char * debug_emoji ="\
        000,000,000,000,128\n\
        000,000,000,000,000\n\
        000,000,255,000,000\n\
        000,000,000,000,000\n\
        255,000,000,000,000\n";
    MicroBitImage img(debug_emoji);
    uBit.display.print(img);
 }

void
mic_inference_test()
{
    if (mic == NULL){
        mic = uBit.adc.getChannel(uBit.io.microphone);
        mic->setGain(7,0);          // Uncomment for v1.47.2
        //mic->setGain(7,1);        // Uncomment for v1.46.2
    }

    // alloc inferencing buffers
    inference.buffers[0] = (int8_t *)malloc(EI_CLASSIFIER_SLICE_SIZE * sizeof(int8_t));

    if (inference.buffers[0] == NULL) {
        uBit.serial.printf("Failed to alloc buffer 1\n");
        return;
    }

    inference.buffers[1] = (int8_t *)malloc(EI_CLASSIFIER_SLICE_SIZE * sizeof(int8_t));

    if (inference.buffers[0] == NULL) {
        uBit.serial.printf("Failed to alloc buffer 2\n");
        free(inference.buffers[0]);
        return;
    }

    uBit.serial.printf("Allocated buffers\n");

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = EI_CLASSIFIER_SLICE_SIZE;
    inference.buf_ready = 0;

    mic->output.setBlocking(true);

    if (processor == NULL)
        processor = new StreamNormalizer(mic->output, 0.15f, true, DATASTREAM_FORMAT_8BIT_SIGNED);

    if (streamer == NULL)
        streamer = new ContinuousAudioStreamer(processor->output, &inference);

    uBit.io.runmic.setDigitalValue(1);
    uBit.io.runmic.setHighDrive(true);

    uBit.serial.printf("Allocated everything else\n");

    // number of frames since we heard 'microbit'
    uint8_t last_keywords = 0b0;
    uint8_t last_another_keywords = 0b0;

    int heard_keyword_x_ago = 100;
    int heard_another_keyword_x_ago = 100;
    uBit.io.P0.setDigitalValue(0); // switches off LED at Pin 0 when code first runs
    uBit.io.P1.setDigitalValue(0); // switches off LED at Pin 1 when code first runs

    while(1) {
        uBit.sleep(1);

        if (inference.buf_ready) {
            inference.buf_ready = 0;

            static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);

            signal_t signal;
            signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
            signal.get_data = &microphone_audio_signal_get_data;
            ei_impulse_result_t result = { 0 };

            EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, false);
            if (r != EI_IMPULSE_OK) {
                ei_printf("ERR: Failed to run classifier (%d)\n", r);
                return;
            }

            bool heard_keyword_this_window = false;
            bool heard_another_keyword_this_window = false;

            if (++print_results >= 0) {
                // print the predictions
                ei_printf("Predictions (DSP: %d ms., Classification: %d ms.): \n",
                    result.timing.dsp, result.timing.classification);
                for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                    ei_printf("%s: ", result.classification[ix].label);
                    ei_printf_float(result.classification[ix].value);
                    ei_printf("\n");

                    if (strcmp(result.classification[ix].label, INFERENCING_KEYWORD) == 0 && result.classification[ix].value > 0.4) {
                        heard_keyword_this_window = true;
                    }
                    
                    else if (strcmp(result.classification[ix].label, ANOTHER_KEYWORD) == 0 && result.classification[ix].value > 0.4) {
                        heard_another_keyword_this_window = true;
                    }
                    
                }

                last_keywords = last_keywords << 1 & 0x1f;
                last_another_keywords = last_another_keywords << 1 & 0x1f;

                if (heard_keyword_this_window) {
                    last_keywords += 1;
                }
                else if (heard_another_keyword_this_window) {
                    last_another_keywords += 1;
                }

                uint8_t keyword_count = 0;
                uint8_t another_keyword_count = 0;
                for (size_t ix = 0; ix < 5; ix++) {
                    keyword_count += (last_keywords >> ix) & 0x1;
                }
                for (size_t ix = 0; ix < 5; ix++) {
                    another_keyword_count += (last_another_keywords >> ix) & 0x1;
                }

                if (heard_keyword_this_window) {
                    ei_printf("\nHeard keyword: %s (%d times, needs 5)\n", INFERENCING_KEYWORD, keyword_count);
                    last_keywords = 0;
                    heard_keyword_x_ago = 0;
                }
                else if (heard_another_keyword_this_window) {
                    ei_printf("\nHeard keyword: %s (%d times, needs 5)\n", ANOTHER_KEYWORD, another_keyword_count);
                    last_another_keywords = 0;
                    heard_another_keyword_x_ago = 0;
                }

                if (keyword_count >= 1) {
                    ei_printf("\n\n\nDefinitely heard keyword: \u001b[32m%s\u001b[0m\n\n\n", INFERENCING_KEYWORD);
                    last_keywords = 0;
                    heard_keyword_x_ago = 0;
                }
                else if (another_keyword_count >= 1) {
                    ei_printf("\n\n\nDefinitely heard keyword: \u001b[32m%s\u001b[0m\n\n\n", ANOTHER_KEYWORD);
                    last_another_keywords = 0;
                    heard_another_keyword_x_ago = 0;
                }
                else {
                    heard_keyword_x_ago++;
                    heard_another_keyword_x_ago++;
                }

                if (heard_keyword_x_ago <= 4) {
                    heard_keyword();
                }
                else if (heard_another_keyword_x_ago <= 4) {
                    heard_another_keyword();
                }
                else {
                    heard_other();
                }
            }
        }
    }
}

/**
 * Microbit implementations for Edge Impulse target-specific functions
 */
EI_IMPULSE_ERROR ei_sleep(int32_t time_ms) {
    uBit.sleep(time_ms);
    return EI_IMPULSE_OK;
}

void ei_printf(const char *format, ...) {
    char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    uBit.serial.printf("%s", print_buf);
}
