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


enum GATE_STATE : uint8_t {
    GATE_CLOSED,
    GATE_OPEN,
    GATE_RETRIG
    };
#define GATE_STATE_MAX GATE_RETRIG

enum RETRIGGER_MODE : uint8_t {
    RT_OFF,    // never retrigger
    RT_NEW,    // retrigger when new notes are played
    RT_ALWAYS  // also retrigger when notes are released if other notes are still held
    };
#define RETRIGGER_MODE_MAX RT_ALWAYS


extern RETRIGGER_MODE retrig_mode;

void setRetrigMode(uint8_t value);
void setRetrigModePC(void);
void sendGate(GATE_STATE gate_state);
