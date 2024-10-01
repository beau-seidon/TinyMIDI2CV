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


#include "cv2.h"
#include "midi.h"
#include "paraphonic.h"
#include "gate.h"


volatile CV2_MODE cv2 = CV2_MODWHEEL;


void setCV2Mode(uint8_t mode)
{
        switch((int)mode) {

            case CV2_MODE::CV2_PITCH_BEND:
                cv2 = CV2_PITCH_BEND;
                break;

            case CV2_MODE::CV2_MODWHEEL:
                cv2 = CV2_CC;
                setMIDICCFilter(MODWHEEL_CC);
                break;

            case CV2_MODE::CV2_CC:
                cv2 = CV2_CC;
                setMIDICCFilter(DEFAULT_CC);
                break;

            case CV2_MODE::CV2_VELOCITY:
                cv2 = CV2_VELOCITY;
                break;

            case CV2_MODE::CV2_NOTE:
                cv2 = CV2_NOTE;
                break;

            case CV2_MODE::CV2_NOTE_INV:
                cv2 = CV2_NOTE_INV;
                break;

            case CV2_MODE::CV2_GATE:
                cv2 = CV2_GATE;
                break;

            case CV2_MODE::CV2_GATE_INV:
                cv2 = CV2_GATE_INV;
                break;

            case CV2_MODE::CV2_PARAPHONIC:
                cv2 = CV2_PARAPHONIC;
                break;

            case CV2_MODE::CV2_PARA_SET:
                setParaphonicModePC();
                break;

            case CV2_MODE::CV2_RETRIG_SET:
                setRetrigModePC();
                break;

            case CV2_MODE::CV2_UNUSED:
                break;

            case CV2_MODE::CV2_TRIG:
                // cv2 = CV2_TRIG;
                break;

            case CV2_MODE::CV2_ENVELOPE:
                // cv2 = CV2_ENVELOPE;
                break;

            case CV2_MODE::CV2_LFO:
                // cv2 = CV2_LFO;
                break;

            case CV2_MODE::CV2_SYNC:
                // cv2 = CV2_SYNC;
                break;

            default:  // CV2_MODE::CV2_MODWHEEL
                cv2 = CV2_CC;
                setMIDICCFilter(MODWHEEL_CC);
                break;
        }
}


void sendNoteCV2(uint8_t note)
{
    OCR1B = note << 2;
}


void sendNoteCV2Inv(uint8_t note)
{
    OCR1B = OCR1C - (note << 2);
}


void sendGateCV2(uint8_t gate)
{
    if (cv2 == CV2_GATE) {
        if (gate) OCR1B = 255;
        else OCR1B = 0;
        return;
    }

    if (cv2 == CV2_GATE_INV) {
        if (gate) OCR1B = 0;
        else OCR1B = 255;
        return;
    }
}


void handleVelocity(uint8_t vel)
{
    if (cv2 == CV2_VELOCITY) OCR1B = vel << 1;
}


void handleControlChange(uint8_t cc_num, uint8_t cc_value)
{
    if (cv2 == CV2_CC && cc_num == MIDI_CC_FILTER) OCR1B = cc_value << 1;
}


void handlePitchBend(uint8_t data1, uint8_t data2)
{
    if (cv2 == CV2_PITCH_BEND) {
        // unsigned int bend;
        // bend = (unsigned int)data2;  // MSB
        // bend <<= 7;
        // bend |= (unsigned int)data1;  // LSB
        // uint8_t bend_CV = map(bend, BEND_MIN, BEND_MAX, 0, 255);

        uint8_t bend_CV = data2 << 1;  // MSB-only behaves better than map

        OCR1B = bend_CV;
    }
}


void handleProgramChange(uint8_t channel, uint8_t program)
{
    // setCV2Mode(program);
    if (program > 0 && program < PARAPHONIC_MODE_MAX) setParaphonicMode(program);
}
