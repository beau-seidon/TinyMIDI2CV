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


#include "midi.h"
#include "sysex.h"
#include "programchange.h"
#include "cvnote.h"
#include "cv2.h"
#include "gate.h"



void translateChannelMessage(void);



volatile uint8_t midi_cc_filter = 3;

volatile uint8_t midi_message_byte = 0;
volatile uint8_t midi_status = 0x0;
volatile uint8_t midi_channel = 0x0;
volatile uint8_t midi_data1 = 0x0;
volatile uint8_t midi_data2 = 0x0;



void parseMIDI(uint8_t midi_RX)
{
    // Nothing is done to the buffer when a RealTime Category message is received.
    if (midi_RX > 0xF7) return;


    // Buffer is cleared when a System Common Category Status (ie, 0xF0 to 0xF7) is received.
    if ((midi_RX > 0xEF) && (midi_RX < 0xF8)) {
        midi_status = 0x0;
        midi_channel = 0x0;
        midi_message_byte = 0;

        if (midi_RX == 0xF0) startSysExListener();
        if (midi_RX == 0xF7) stopSysExListener(midi_RX);
        if (sysex_listen && !sysex_ignore) handleSysEx(midi_RX);

        return;
    }


    // Buffer stores the status when a Voice Category Status (ie, 0x80 to 0xEF) is received.
    if ((midi_RX > 0x7F) && (midi_RX < 0xF0)) {
        midi_status = midi_RX;
        midi_channel = midi_status & 0x0F;
        midi_message_byte = 1;

        return;
    }


    // Any data bytes are ignored when the buffer is 0.
    if (midi_RX < 0x80) {
        if (!midi_status) return;

        if (midi_message_byte == 1) {
            midi_data1 = midi_RX;
            midi_message_byte = 2;

            if ((midi_status >> 4) == 0xC) handleProgramChange(midi_data1);  // handle program change

            return;
        }

        if (midi_message_byte == 2) {
            midi_data2 = midi_RX;
            midi_message_byte = 1;

            translateChannelMessage();

            return;
        }
    }
}



void translateChannelMessage()
{
    if ((midi_channel == MIDI_CHANNEL_FILTER) || MIDI_OMNI) {

        // Handle notes
        if (((midi_status >> 4) == 0x8) || ((midi_status >> 4) == 0x9)) {

            midi_data1 = limitNoteRange(midi_data1);

            if (((midi_status >> 4) == 0x9) && (midi_data2 > 0x0)) {
                handleNoteOn(midi_data1);
                handleVelocity(midi_data2);
            }

            if (((midi_status >> 4) == 0x8) ||
                (((midi_status >> 4) == 0x9) && (midi_data2 == 0x0))) {  // Note on with velocity 0 is note off
                handleNoteOff(midi_data1);
            }
        }


        // Handle control change
        if ((midi_status >> 4) == 0xB) {
            if (midi_data1 == RETRIG_MODE_CC) {
                setRetrigModeCC(midi_data2);
            } else if (cv2 == CC) {
                handleControlChange(midi_data1, midi_data2);
            }
        }


        // Handle pitch bend
        if ((cv2 == PITCH_BEND) && (midi_status >> 4) == 0xE) {
            handlePitchBend(midi_data1, midi_data2);
        }
    }
}
