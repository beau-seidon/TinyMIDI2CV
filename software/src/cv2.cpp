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


#include "cv2.h"
#include "midi.h"
#include "paraphonic.h"
#include "gate.h"



volatile CV2_MODE cv2 = MODWHEEL;



void setCV2Mode(uint8_t mode)
{
        switch((int)mode) {

            case CV2_MODE::PITCH_BEND:
                cv2 = PITCH_BEND;
                break;

            case CV2_MODE::MODWHEEL:
                cv2 = CC;
                midi_cc_filter = 1;
                break;

            case CV2_MODE::CC:
                cv2 = CC;
                midi_cc_filter = 3;
                break;

            case CV2_MODE::VELOCITY:
                cv2 = VELOCITY;
                break;

            case CV2_MODE::NOTE:
                cv2 = NOTE;
                break;

            case CV2_MODE::INV_NOTE:
                cv2 = INV_NOTE;
                break;

            case CV2_MODE::GATE:
                cv2 = GATE;
                break;

            case CV2_MODE::INV_GATE:
                cv2 = INV_GATE;
                break;

            case CV2_MODE::TRIG:
                cv2 = TRIG;
                break;

            case CV2_MODE::ENVELOPE:
                cv2 = ENVELOPE;
                break;

            case CV2_MODE::LFO:
                cv2 = LFO;
                break;

            case CV2_MODE::SYNC:
                cv2 = SYNC;
                break;

            case CV2_MODE::PARAPHONIC:
                cv2 = PARAPHONIC;
                break;

            case CV2_MODE::PARA_SET:
                setParaphonicModePC();
                break;

            case CV2_MODE::RETRIG_SET:
                setRetrigModePC();
                break;

            default:  // CV2_MODE::MODWHEEL
                cv2 = CC;
                midi_cc_filter = 1;
                break;
        }
}



void sendInvNote(uint8_t note)
{
    OCR1B = OCR1C - (note << 2);
}



void sendGateCV(bool gate)
{
    if (cv2 == GATE) {
        if (gate) OCR1B = 255;
        else OCR1B = 0;
        return;
    }

    if (cv2 == INV_GATE) {
        if (gate) OCR1B = 0;
        else OCR1B = 255;
        return;
    }
}



void handleVelocity(uint8_t vel)
{
    if (cv2 == VELOCITY) OCR1B = vel << 1;
}



void handleControlChange(uint8_t cc_num, uint8_t cc_value)
{
    if (cc_num == midi_cc_filter) OCR1B = cc_value << 1;
}



void handlePitchBend(uint8_t data1, uint8_t data2)
{
    // unsigned int bend;
    // bend = (unsigned int)data2;  // MSB
    // bend <<= 7;
    // bend |= (unsigned int)data1;  // LSB
    // uint8_t bend_CV = map(bend, BEND_MIN, BEND_MAX, 0, 255);

    uint8_t bend_CV = data2 << 1;  // MSB-only behaves better than map

    OCR1B = bend_CV;
}
