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


enum PARAPHONIC_MODE : uint8_t {
    PARA_RECENT,        // newest note is played on cv1, previous note is played on cv2
    PARA_RECENT_INV,    // newest note is played on cv2, previous note is played on cv1
    PARA_RECENT_LO,     // lower of the two newest notes is played on cv2, higher is played on cv1
    PARA_RECENT_HI,     // higher of the two newest notes is played on cv2, lower is played on cv1
    PARA_OUTER,         // highest note is played on cv1, lowest note is played on cv2
    PARA_OUTER_INV,     // highest note is played on cv2, lowest note is played on cv1
    PARA_LO,            // lowest pitch note is played on cv2, newest note (if not lo) is played on cv1
    PARA_HI,            // highest pitch note is played on cv2, newest note (if not hi) is played on cv1
    PARA_PEDAL,         // first note is always played on cv2, newest note is played on cv1
    };
#define PARAPHONIC_MODE_MAX PARA_PEDAL


extern PARAPHONIC_MODE para_mode;

void setParaphonicMode(uint8_t value);
void setParaphonicModePC(void);
void handleParaPriority(volatile uint8_t *note_buffer, volatile uint8_t active_notes);
