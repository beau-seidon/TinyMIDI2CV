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


#include "cvnote.h"
#include "cv2.h"
#include "paraphonic.h"
#include "gate.h"



volatile uint8_t note_buffer[MAX_NOTES] = {0};
volatile uint8_t active_notes = 0;                  // Gate will be open while any keys are pressed (any notes active)

const uint8_t LOW_NOTE = 36;                        // Any note lower than C2 will be interpreted as C2
const uint8_t HIGH_NOTE = 96;                       // Any note higher than C7 will be interpreted as C7



uint8_t limitNoteRange(uint8_t note)
{
    if (note < LOW_NOTE) note = LOW_NOTE;    // If note is lower than C2 set it to C2
    if (note > HIGH_NOTE) note = HIGH_NOTE;  // If note is higher than C7 set it to C7
    note -= LOW_NOTE;                        // Subtract 36 to get into CV range
    return note;
}



void handleNoteOn(uint8_t note)
{
    for (uint8_t n = 0; n < active_notes; n++) {  // If note is already in the buffer, play it, but don't add it again
        if (note_buffer[n] == note) {
            sendNote(note);
            return;
        }
    }

    if (active_notes < MAX_NOTES) {
        note_buffer[active_notes] = note;
        active_notes++;

        if (cv2 == PARAPHONIC) {
            if (active_notes > 1) {
                handleParaPriority();
            } else {
                sendNote(note);
                sendParaNote(note);
            }
        } else {
            sendNote(note);
        }

        if (retrig_mode != RT_OFF)
            sendGate(GATE_STATE::RETRIG);
        else
            sendGate(GATE_STATE::OPEN);
    }
}



void handleNoteOff(uint8_t note)
{
    if (!active_notes) return;

    bool note_off_match = false;

    for (uint8_t n = 0; n < active_notes; n++) {  // Check buffer to see if note is active
        if (note_buffer[n] == note) {
            note_off_match = true;
            if (n < (MAX_NOTES-1)) {  // If note is removed from middle of buffer, shift all notes to prevent empty slots
                note_buffer[n] = note_buffer[n+1];
                note_buffer[n+1] = note;
            }
        }
    }

    if (note_off_match) {
        note_off_match = false;
        note_buffer[active_notes] = 0;  // Remove requested note and play next in buffer
        active_notes--;

        if (active_notes) {
            note = note_buffer[active_notes-1];

            if (cv2 == PARAPHONIC) {
                if (active_notes > 1) {
                    handleParaPriority();
                } else {
                    sendNote(note);
                    sendParaNote(note);
                }
            } else {
                sendNote(note);
            }

            if (retrig_mode == RT_ALWAYS)
                sendGate(GATE_STATE::RETRIG);
            else
                sendGate(GATE_STATE::OPEN);

        } else {
            sendGate(GATE_STATE::CLOSED);
        }
    }
}



void sendNote(uint8_t note)
{
    OCR1A = note << 2;  // Multiply note by 4 to set the voltage (1v/octave)
    if (cv2 == NOTE) OCR1B = note << 2;
    if (cv2 == INV_NOTE) sendInvNote(note);
}
