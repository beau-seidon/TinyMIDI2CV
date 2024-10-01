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


#include "gate.h"
#include "cv2.h"


RETRIGGER_MODE retrig_mode = RT_NEW;


void setRetrigMode(uint8_t value)
{
    retrig_mode = (RETRIGGER_MODE)value;
}


void setRetrigModePC()
{
    switch(retrig_mode) {
        case RT_NEW:
            retrig_mode = RT_ALWAYS;
            break;
        case RT_ALWAYS:
            retrig_mode = RT_OFF;
            break;
        case RT_OFF:
        default:
            retrig_mode = RT_NEW;
            break;
    }
}


// void setRetrigModeCC(uint8_t cc_value)
// {
//     if (cc_value > 83)
//         retrig_mode = RT_ALWAYS;
//     else if (cc_value > 42 && cc_value < 84)
//         retrig_mode = RT_NEW;
//     else
//         retrig_mode = RT_OFF;
// }


void sendGate(GATE_STATE gate_state)
{
    switch (gate_state) {
        case GATE_OPEN:
            digitalWrite(GATE_CV_PIN, HIGH);  // Set Gate HIGH
            sendGateCV2(HIGH);
            break;

        case GATE_RETRIG:
            digitalWrite(GATE_CV_PIN, LOW);  // Set Gate LOW
            sendGateCV2(LOW);
            delayMicroseconds(32);
            digitalWrite(GATE_CV_PIN, HIGH);
            sendGateCV2(HIGH);
            break;

        case GATE_CLOSED:
        default:
            digitalWrite(GATE_CV_PIN, LOW);
            sendGateCV2(LOW);
            break;
    }
}


// for future use, not tested yet:
/*
void sendTrig()
{
    if (active_notes > 0) {
        if(S_TRIG_MODE) {
            digitalWrite(TRIG_CV_PIN, LOW);
            delayMicroseconds(32);
            digitalWrite(TRIG_CV_PIN, HIGH);
        } else {
            digitalWrite(TRIG_CV_PIN, HIGH);
            delayMicroseconds(32);
            digitalWrite(TRIG_CV_PIN, LOW);
        }
    }
}
*/
