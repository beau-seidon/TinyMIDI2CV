/*
    TinyMIDI2CV

    Copyright 2023-2024 Beau Sterling (Aether Soundlab)

    Hardware config is based on DIY Good Ol’ MIDI to CV by Jan Ostman:
        (*) All in the spirit of open-source and open-hardware
        Janost 2019 Sweden
        The goMIDI2CV interface
        http://blog.dspsynth.eu/diy-good-ol-midi-to-cv/
        Copyright 2019 DSP Synthesizers Sweden.


    This file is part of TinyMIDI2CV.

    TinyMIDI2CV is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    TinyMIDI2CV is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
    more details.

    You should have received a copy of the GNU General Public License along with
    TinyMIDI2CV. If not, see <https://www.gnu.org/licenses/>.
*/


#include "sysex.h"
#include "midi.h"
#include "cv2.h"
#include "paraphonic.h"
#include "gate.h"


static inline void interpretSysExCommand(volatile uint8_t *sysex_buffer);


#define TM2CV_SYSEX_ID 0x7D   // "Special ID" for non-commercial use only
#define TM2CV_DEVICE_ID 0x42  // ASCII * (wild card), could mean anything... eg: life, the universe, and everything

#define SYSEX_BUFFER_SIZE 6
static volatile uint8_t sysex_buffer[SYSEX_BUFFER_SIZE];
static volatile uint8_t sysex_byte_count = 0;

static volatile bool sysex_msg_ok = false;

volatile bool sysex_listen = false;
volatile bool sysex_ignore = false;


void resetSysExBuffer(int max_element)
{
    for (int i = 0; i < max_element; ++i) {
        sysex_buffer[i] = 0x0;
    }
    sysex_byte_count = 0;
    sysex_ignore = true;
}


void startSysExListener(uint8_t syx)
{
    resetSysExBuffer(SYSEX_BUFFER_SIZE);
    sysex_listen = true;
    sysex_ignore = false;
    sysex_msg_ok = false;
    handleSysExData(syx);
}


void stopSysExListener(uint8_t syx)
{
    handleSysExData(syx);
    resetSysExBuffer(SYSEX_BUFFER_SIZE);
    sysex_listen = false;
    sysex_ignore = false;
    sysex_msg_ok = false;
}


void handleSysExData(uint8_t syx)
{
    if (!sysex_ignore) {
        // check for buffer overflow
        if (sysex_byte_count >= SYSEX_BUFFER_SIZE) {
            resetSysExBuffer(SYSEX_BUFFER_SIZE);
            return;  // reset entire buffer and ignore
        }

        // load buffer with new byte
        sysex_buffer[sysex_byte_count++] = syx;

        // check to see if message is valid
        if (!sysex_msg_ok) {
            if (sysex_buffer[0] == 0xF0) {  // if valid sysex start byte
                if (sysex_byte_count >= 3) {
                    if (sysex_buffer[SYX_SYXID] == TM2CV_SYSEX_ID &&
                        sysex_buffer[SYX_DEVID] == TM2CV_DEVICE_ID) {  // if valid address
                        sysex_msg_ok = true;
                        return;  // message ok, load the rest
                    } else {
                        resetSysExBuffer(SYSEX_BUFFER_SIZE);
                        return;  // invalid address, reset and ignore buffer
                    }
                } else { return; }  // not enough bytes yet
            } else {
                resetSysExBuffer(SYSEX_BUFFER_SIZE);
                return;  // invalid start byte, reset and ignore buffer
            }
        }
    }

    // interpret sysex message
    if (syx == 0xF7 && sysex_msg_ok) {  // if current byte is end of message flag
        interpretSysExCommand(sysex_buffer);
    }
}


static inline void interpretSysExCommand(volatile uint8_t *sysex_buffer)
{
    // example message:
    // F0 7D 42 03 05 xx... 7F
    // means set CV2 mode to CV2_NOTE_INV
    // format: START SYXID DEVID CMD VALUE extrastuff? STOP

    if (sysex_ignore) return;
    uint8_t value = sysex_buffer[SYX_VALUE];

    switch(sysex_buffer[SYX_COMMAND]) {
        case SYX_SET_CHANNEL:
            if (value < 0 || value > 16) break;  // 16 = omni
            setMIDIChannelFilter(value);
            break;

        case SYX_SET_CC_FILTER:
            if (value < 0 || value > 127) break;
            setMIDICCFilter(value);
            break;

        case SYX_SET_CV2_MODE:
            if (value < 0 || value > CV2_MODE_MAX) break;
            setCV2Mode(value);
            break;

        case SYX_SET_PARAPHONIC_MODE:
            if (value < 0 || value > PARAPHONIC_MODE_MAX) break;
            setParaphonicMode(value);
            break;

        case SYX_SET_RETRIG_MODE:
            if (value < 0 || value > RETRIGGER_MODE_MAX) break;
            setRetrigMode(value);
            break;

        default:
            break;
    }
}
