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

enum SYSEX_BYTE_ID : uint8_t {
    SYX_START,
    SYX_SYXID,
    SYX_DEVID,
    SYX_COMMAND,
    SYX_VALUE,
    SYX_STOP
    };
#define SYSEX_BYTE_ID_MAX SYX_STOP

enum SYSEX_COMMAND : uint8_t {
    SYX_SET_CHANNEL = 1,
    SYX_SET_CC_FILTER,
    SYX_SET_CV2_MODE,
    SYX_SET_PARAPHONIC_MODE,
    SYX_SET_RETRIG_MODE
    };
#define SYSEX_COMMAND_MAX SYX_SET_RETRIG_MODE


extern volatile bool sysex_listen;
extern volatile bool sysex_ignore;

void startSysExListener(uint8_t syx);
void stopSysExListener(uint8_t syx);
void handleSysExData(uint8_t syx);
