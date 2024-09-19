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


#pragma once

#include <Arduino.h>
#include "global.h"


enum CV2_MODE {  // set by Program Change or SysEx
    PITCH_BEND,  MODWHEEL,  CC,          VELOCITY,
    NOTE,        INV_NOTE,  GATE,        INV_GATE,
    PARAPHONIC,  PARA_SET,  RETRIG_SET,  UNUSED,
    TRIG,        ENVELOPE,  LFO,         SYNC
};


extern volatile CV2_MODE cv2;

void setCV2Mode(uint8_t mode);
void sendInvNote(uint8_t note);
void sendGateCV(bool gate);
void handleVelocity(uint8_t vel);
void handleControlChange(uint8_t cc_num, uint8_t cc_value);
void handlePitchBend(uint8_t data1, uint8_t data2);
