/*
    TinyMIDI2CV

    Copyright 2023-2024 Beau Sterling (Aether Soundlab)

    Based on DIY Good Ol’ MIDI to CV by Jan Ostman:
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



void handleSysExCommand(volatile uint8_t *sysex_buffer);



#define SYSEX_ID 0x7D   // "Special ID" for non-commercial use only
#define DEVICE_ID 0x42  // ASCII * (wild card), could mean anything... eg: life, the universe, and everything

enum SYSEX_BYTE_ID {
    START, SYXID, DEVID, PADDING,
    COMMAND, VALUE, CHECKSUM, STOP
    };

enum SYSEX_COMMAND {
    SYX_SET_CHANNEL, SYX_SET_CV2_MODE, SYX_SET_CC_FILTER, SYX_SET_PARAPHONIC_MODE, SYX_SET_RETRIG_MODE
    };

#define SYSEX_BUFFER_SIZE 16
volatile uint8_t sysex_buffer[SYSEX_BUFFER_SIZE];
volatile uint8_t sysex_byte_count = 0;

volatile bool sysex_listen = false;
volatile bool sysex_ignore = false;
volatile bool sysex_msg_ok = false;



void startSysExListener()
{
    sysex_listen = true;
}



void stopSysExListener(uint8_t syx)
{
    handleSysEx(syx);
    for (int i = 0; i < SYSEX_BUFFER_SIZE; ++i) {
        sysex_buffer[i] = 0x0;
    }
    sysex_byte_count = 0;
    sysex_listen = false;
    sysex_ignore = false;
    sysex_msg_ok = false;
}



void handleSysEx(uint8_t syx)
{
    // check for buffer overflow
    if (sysex_byte_count >= SYSEX_BUFFER_SIZE) {
        for (int i = 0; i < SYSEX_BUFFER_SIZE; ++i) {
            sysex_buffer[i] = 0x0;
        }
        sysex_byte_count = 0;
        sysex_ignore = true;
        return;  // reset entire buffer and ignore
    }


    // load buffer with new byte
    sysex_buffer[sysex_byte_count] = syx;
    sysex_byte_count++;


    // check to see if message is valid
    if (!sysex_msg_ok) {
        if (sysex_buffer[0] == 0xF0) {  // if valid sysex start byte
            if (sysex_byte_count >= 3) {
                if (sysex_buffer[1] == SYSEX_ID || sysex_buffer[2] == DEVICE_ID) {  // if valid address
                    sysex_msg_ok = true;
                    return;  // message ok, load the rest
                } else {
                    for (int i = 0; i < sysex_byte_count; ++i) {
                        sysex_buffer[i] = 0x0;
                    }
                    sysex_byte_count = 0;
                    sysex_ignore = true;
                    return;  // invalid address, reset and ignore buffer
                }
            } else { return; }  // not enough bytes yet
        } else {
            sysex_buffer[0] = 0x0;
            sysex_byte_count = 0;
            sysex_ignore = true;
            return;  // invalid start byte, reset and ignore buffer
        }
    }


    // interpret sysex message
    if (syx == 0xF7) {  // if current byte is end of message flag
        handleSysExCommand(sysex_buffer);
    }

}



void handleSysExCommand(volatile uint8_t *sysex_buffer)
{
    // example message:
    // F0 7D 42 00 02 05 00 7F
    // means set CV2 mode to INV_NOTE

    uint8_t value = sysex_buffer[SYSEX_BYTE_ID::VALUE];

    switch(sysex_buffer[SYSEX_BYTE_ID::COMMAND]) {
        case SYX_SET_CHANNEL:
            if (value < 0 || value > 15) break;
            midi_channel = value;
            break;

        case SYX_SET_CV2_MODE:
            if (value < 0 || value > 15) break;
            setCV2Mode(value);
            break;

        case SYX_SET_CC_FILTER:
            if (value < 0 || value > 127) break;
            midi_cc_filter = value;
            break;

        case SYX_SET_PARAPHONIC_MODE:
            if (value < 0 || value > 6) break;
            para_mode = (PARAPHONIC_MODE)value;
            break;

        case SYX_SET_RETRIG_MODE:
            if (value < 0 || value > 2) break;
            retrig_mode = (RETRIGGER_MODE)value;
            break;

        default:
            break;
    }
}
