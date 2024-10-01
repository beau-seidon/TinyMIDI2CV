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


#include "midi.h"
#include "sysex.h"
#include "cvnote.h"
#include "cv2.h"
#include "gate.h"


static inline void translateChannelMessage(uint8_t status, uint8_t channel, uint8_t data1, uint8_t data2);

volatile uint8_t MIDI_OMNI = 0;              // Set 1 to ignore filter, or 0 to use a single MIDI channel
volatile uint8_t MIDI_CHANNEL_FILTER = 0x0;  // MIDI channel the ATTiny listens for; default is channel 1 (zero-indexed, 0x0-0xF); can by changed by SysEx
volatile uint8_t MIDI_CC_FILTER = 3;         // MIDI CC number used for CV2 output; can be changed by SysEx

static volatile uint8_t midi_status = 0x0;
static volatile uint8_t midi_channel = 0x0;
static volatile uint8_t midi_data1 = 0x0;
static volatile uint8_t midi_data2 = 0x0;
static volatile uint8_t byte_count = 0;


void parseMIDI(uint8_t message_byte)
{
    // Nothing is done to the buffer when a RealTime Category message is received.
    if (message_byte > 0xF7) return;


    // Buffer is cleared when a System Common Category Status (ie, 0xF0 to 0xF7) is received.
    if (message_byte > 0xEF && message_byte < 0xF8) {
        midi_status = 0x0;
        midi_channel = 0x0;
        byte_count = 0;

        if (message_byte == 0xF0) startSysExListener(message_byte);
        if (message_byte == 0xF7) stopSysExListener(message_byte);

        return;
    }

    if (sysex_listen) {  // disregard all other messages until sysex is done (this does not follow MIDI spec)
        handleSysExData(message_byte);
    }

    else {
        // Buffer stores the status when a Voice Category Status (ie, 0x80 to 0xEF) is received.
        if (message_byte > 0x7F && message_byte < 0xF0) {
            midi_status = message_byte;
            midi_channel = midi_status & 0x0F;
            byte_count = 1;

            return;
        }

        // Any data bytes are ignored when the buffer is 0.
        if (message_byte < 0x80) {
            if (!midi_status) return;

            if (byte_count == 1) {
                midi_data1 = message_byte;
                byte_count = 2;

                if ((midi_status >> 4) == 0xC &&
                    (midi_channel == MIDI_CHANNEL_FILTER || MIDI_OMNI)) {
                    handleProgramChange(midi_channel, midi_data1);
                }

                return;
            }

            if (byte_count == 2) {
                midi_data2 = message_byte;
                byte_count = 1;

                translateChannelMessage(midi_status, midi_channel, midi_data1, midi_data2);

                return;
            }
        }
    }
}


static inline void translateChannelMessage(uint8_t status, uint8_t channel, uint8_t data1, uint8_t data2)
{
    if (channel == MIDI_CHANNEL_FILTER || MIDI_OMNI) {

        if ((status >> 4) == 0x8 || (status >> 4) == 0x9) {  // Handle notes

            data1 = limitNoteRange(data1);

            if ((status >> 4) == 0x9 && data2 > 0x0) {
                handleNoteOn(data1);
                handleVelocity(data2);
            }

            if ((status >> 4) == 0x8 ||
                ((status >> 4) == 0x9 && data2 == 0x0)) {  // Note on with velocity 0 is note off
                handleNoteOff(data1);
            }
        }

        if ((status >> 4) == 0xB) handleControlChange(data1, data2);
        if ((status >> 4) == 0xE) handlePitchBend(data1, data2);
    }
}


void setMIDIChannelFilter(uint8_t channel)
{
    if (channel > 15) {
        MIDI_OMNI = 1;
    } else {
        MIDI_OMNI = 0;
        MIDI_CHANNEL_FILTER = channel;
    }
}


void setMIDICCFilter(uint8_t cc)
{
    MIDI_CC_FILTER = cc;
}
