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


#pragma once

#include <Arduino.h>
#include "global.h"


// set by Program Change or SysEx
enum CV2_MODE : uint8_t {
    CV2_PITCH_BEND,     // outputs 0-5 V, useful when summed with CV1 voltage
    CV2_MODWHEEL,       // outputs 0-5 V depending on value of CC 1
    CV2_CC,             // outputs 0-5 V depending on value of active CC filter
    CV2_VELOCITY,       // outputs 0-5 V depending on last note's velocity

    CV2_NOTE,           // plays note CV 1V/oct
    CV2_NOTE_INV,       // plays note CV 1V/oct, with backwards keyboard
    CV2_GATE,           // normally low gate, note on sends gate high
    CV2_GATE_INV,       // normally high gate, note on sends gate low

    CV2_PARAPHONIC,     // use paraphonic mode to determine note output
    CV2_PARA_SET,       // cycle through paraphonic modes
    CV2_RETRIG_SET,     // cycle through retrigger modes
    CV2_UNUSED,         // unused

    CV2_TRIG,           // not implemented yet
    CV2_ENVELOPE,       // not implemented yet
    CV2_LFO,            // not implemented yet
    CV2_SYNC            // not implemented yet
};
#define CV2_MODE_MAX CV2_SYNC

#define DEFAULT_CC 3  // lowest CC# undefined by MIDI spec
#define MODWHEEL_CC 1


extern volatile CV2_MODE cv2;

void setCV2Mode(uint8_t mode);
void sendNoteCV2(uint8_t note);
void sendNoteCV2Inv(uint8_t note);
void sendGateCV2(uint8_t gate);
void handleVelocity(uint8_t vel);
void handleControlChange(uint8_t cc_num, uint8_t cc_value);
void handlePitchBend(uint8_t data1, uint8_t data2);
void handleProgramChange(uint8_t channel, uint8_t program);
