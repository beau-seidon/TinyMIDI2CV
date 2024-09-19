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


#include "paraphonic.h"
#include "cvnote.h"



PARAPHONIC_MODE para_mode = PARA_PEDAL;



void setParaphonicModePC()
{
    int pm = para_mode;
    if (pm >= 6) {
        pm = 0;
    } else {
        ++pm;
    }
    para_mode = (PARAPHONIC_MODE)pm;
}



void handleParaPriority()
{
    if (active_notes < 2) return;
    uint8_t new_note = note_buffer[active_notes - 1];
    uint8_t last_note = note_buffer[active_notes - 2];
    uint8_t pedal_note = note_buffer[0];
    uint8_t lo_note = 127;
    uint8_t hi_note = 0;

    for (int i = 0; i < active_notes; ++i) {
        if (note_buffer[i] > hi_note) hi_note = note_buffer[i];
        if (note_buffer[i] < lo_note) lo_note = note_buffer[i];
    }

    switch (para_mode) {
        case PARA_RECENT:
            sendNote(new_note);
            sendParaNote(last_note);
            break;

        case PARA_RECENT_LO:
            if (new_note > last_note) {
                sendNote(new_note);
                sendParaNote(last_note);
            } else {
                sendNote(last_note);
                sendParaNote(new_note);
            }
            break;

        case PARA_RECENT_HI:
            if (new_note > last_note) {
                sendNote(last_note);
                sendParaNote(new_note);
            } else {
                sendNote(new_note);
                sendParaNote(last_note);
            }
            break;

        case PARA_OUTER:
            sendNote(hi_note);
            sendParaNote(lo_note);
            break;

        case PARA_HI:
            if (new_note >= hi_note) {
                sendNote(last_note);
                sendParaNote(new_note);
            } else {
                sendNote(new_note);
                sendParaNote(hi_note);
            }
            break;

        case PARA_LO:
            if (new_note <= lo_note) {
                sendNote(last_note);
                sendParaNote(new_note);
            } else {
                sendNote(new_note);
                sendParaNote(lo_note);
            }
            break;

        case PARA_PEDAL:
        default:
            sendNote(new_note);
            sendParaNote(pedal_note);
            break;
    }
}



void sendParaNote(uint8_t note)
{
    OCR1B = note << 2;
}
