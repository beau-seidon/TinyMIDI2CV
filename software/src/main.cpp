// (*) All in the spirit of open-source and open-hardware
// Janost 2019 Sweden
// The goMIDI2CV interface
// http://blog.dspsynth.eu/diy-good-ol-midi-to-cv/
// Copyright 2019 DSP Synthesizers Sweden.
// Changes Copyright 2023-2024 Beau Sterling, Aether Soundlab, USA
//
// Authors: Jan Ostman and Beau Sterling
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.


// Set Fuses to E1 DD FE for PLLCLK 16MHz



#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>



#ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif



const int USI_PIN = 0;
const int NOTE_CV_PIN = 1;
const int GATE_CV_PIN = 2;
const int MISC_CV_PIN = 4;

volatile uint8_t midi_message_byte = 0;
volatile uint8_t midi_status = 0x0;
volatile uint8_t midi_channel = 0x0;
volatile uint8_t midi_data1 = 0x0;
volatile uint8_t midi_data2 = 0x0;

const bool MIDI_OMNI = false;                       // Set true to ignore filter, or false to use a single midi channel
const uint8_t MIDI_CHANNEL_FILTER = 0;              // MIDI channel 1
const uint8_t MIDI_CC_FILTER = 3;

const uint8_t LOW_NOTE = 36;                        // Any note lower than C2 will be interpreted as C2
const uint8_t HIGH_NOTE = 96;                       // Any note higher than C7 will be interpreted as C7

const uint8_t POLYPHONY_MAX = 16;                   // Set buffer and gate "polyphony" limit
volatile uint8_t note_buffer[POLYPHONY_MAX] = {0};
volatile uint8_t active_notes = 0;                  // Gate will be open while any keys are pressed (any notes active)

const int BEND_MAX = 0x3FFF;
const int BEND_CENTER = 0x2000;
const int BEND_MIN = 0x0;

enum CV2_MODE { UNUSED, CC, PITCH_BEND, VELOCITY, ENVELOPE, GATE, TRIG, SYNC, NOTE, INV_NOTE, LFO };
volatile CV2_MODE CV2 = VELOCITY;

volatile bool RETRIGGER = true;
const uint8_t RETRIG_MODE_CC = 68;
enum GATE_STATE { CLOSED, OPEN, RETRIG };


void setup()
{
    // Enable 64 MHz PLL and use as source for Timer1
    PLLCSR = 1 << PCKE | 1 << PLLE;

    // Setup the USI
    USICR = 0;                                     // Disable USI.
    GIFR = 1 << PCIF;                              // Clear pin change interrupt flag.
    GIMSK |= 1 << PCIE;                            // Enable pin change interrupts
    PCMSK |= 1 << PCINT0;                          // Enable pin change on pin 0

    // Set up Timer/Counter1 for PWM output
    TIMSK = 0;                                     // Timer interrupts OFF
    TCCR1 = 1 << PWM1A | 2 << COM1A0 | 1 << CS10;  // PWM A, clear on match, 1:1 prescale
    OCR1A = 0;                                     // Set initial Pitch to C2
    OCR1B = 127;                                   // Set initial bend to center
    OCR1C = 239;                                   // Set count to semi tones

    GTCCR = 0;
    GTCCR = 1 << PWM1B | 2 << COM1B0;              // PWM B, clear on match

    // Setup GPIO
    pinMode(USI_PIN, INPUT);                       // Enable USI input pin
    pinMode(NOTE_CV_PIN, OUTPUT);                  // Enable PWM output pin
    pinMode(GATE_CV_PIN, OUTPUT);                  // Enable Gate output pin
    pinMode(MISC_CV_PIN, OUTPUT);                  // Enable Pitchbend PWM output pin

    digitalWrite(GATE_CV_PIN,LOW);                 // Set initial Gate to LOW;
}



void handleProgramChange(uint8_t program)
{
    if ((midi_channel == MIDI_CHANNEL_FILTER) || MIDI_OMNI) {

        switch((int)program) {

            case CV2_MODE::UNUSED:
                CV2 = UNUSED;
                break;

            case CV2_MODE::CC:
                CV2 = CC;
                break;

            case CV2_MODE::PITCH_BEND:
                CV2 = PITCH_BEND;
                break;

            case CV2_MODE::VELOCITY:
                CV2 = VELOCITY;
                break;

            case CV2_MODE::ENVELOPE:
                CV2 = ENVELOPE;
                break;

            case CV2_MODE::GATE:
                CV2 = GATE;
                break;

            case CV2_MODE::TRIG:
                CV2 = TRIG;
                break;

            case CV2_MODE::SYNC:
                CV2 = SYNC;
                break;

            case CV2_MODE::NOTE:
                CV2 = NOTE;
                break;

            case CV2_MODE::INV_NOTE:
                CV2 = INV_NOTE;
                break;

            case CV2_MODE::LFO:
                CV2 = LFO;
                break;

            default:  // ISSUE#1: playing notes while wailing on pitch bend seems to send program here
                CV2 = UNUSED;
                break;
        }
    }
}



void limitNoteRange()
{
    if (midi_data1 < LOW_NOTE) midi_data1 = LOW_NOTE;    // If note is lower than C2 set it to C2
    if (midi_data1 > HIGH_NOTE) midi_data1 = HIGH_NOTE;  // If note is higher than C7 set it to C7
    midi_data1 -= LOW_NOTE;                              // Subtract 36 to get into CV range
}



void sendNote(uint8_t note)
{
    OCR1A = note << 2;  // Multiply note by 4 to set the voltage (1v/octave)
}



void sendInvNote(uint8_t note)
{
    OCR1B = OCR1C - (note << 2);
}



void handleRetrigModeToggle(uint8_t cc_value)
{
    if (cc_value > 63)
        RETRIGGER = true;
    else
        RETRIGGER = false;
}



void sendGate(GATE_STATE gate_state)
{
    switch (gate_state) {

        case GATE_STATE::CLOSED:
            digitalWrite(GATE_CV_PIN, LOW);  // Set Gate LOW
            break;

        case GATE_STATE::OPEN:
            digitalWrite(GATE_CV_PIN, HIGH);  // Set Gate HIGH
            break;

        case GATE_STATE::RETRIG:
            digitalWrite(GATE_CV_PIN, LOW);
            delayMicroseconds(32);
            digitalWrite(GATE_CV_PIN, HIGH);
            break;

        default:
            digitalWrite(GATE_CV_PIN, LOW);
            break;
    }
}



void handleVelocity(uint8_t vel)
{
    if (CV2 == VELOCITY) OCR1B = vel << 1;
}



void handleControlChange(uint8_t cc_num, uint8_t cc_value)
{
    if (cc_num == MIDI_CC_FILTER) OCR1B = cc_value << 1;
}



void handlePitchBend(uint8_t data1, uint8_t data2)  // test more: buggy with cakewalk / voltage modular
{
    unsigned int bend;
    bend = (unsigned int)data2;  // MSB
    bend <<= 7;
    bend |= (unsigned int)data1;  // LSB

    uint8_t bend_CV = map(bend, BEND_MIN, BEND_MAX, 0, 255);
    OCR1B = bend_CV;  // Output pitchbend CV
}



void handleNoteOn(uint8_t note)
{
    for (uint8_t n = 0; n < active_notes; n++) {  // If note is already in the buffer, play it, but don't add it again
        if (note_buffer[n] == note) {
            sendNote(note);
            // if (CV2 == INV_NOTE) sendInvNote(note);  // not tested
            return;
        }
    }

    if (active_notes < POLYPHONY_MAX) {
        note_buffer[active_notes] = note;
        active_notes++;

        sendNote(note);
        // if (CV2 == INV_NOTE) sendInvNote(note);  // not tested

        if (RETRIGGER)
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
            if (n < (POLYPHONY_MAX-1)) {  // If note is removed from middle of buffer, shift all notes to prevent empty slots
                note_buffer[n] = note_buffer[n+1];
                note_buffer[n+1] = note;
            }
        }
    }

    if (note_off_match) {
        note_off_match = false;
        active_notes--;

        note_buffer[active_notes+1] = 0;  // Remove requested note and play next in buffer

        if (active_notes) {
            sendNote(note_buffer[active_notes-1]);
            // if (CV2 == INV_NOTE) sendInvNote(note_buffer[active_notes-1]);  // not tested

            sendGate(GATE_STATE::OPEN);

        } else {
            sendGate(GATE_STATE::CLOSED);
        }
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



void translateChannelMessage()
{
    if ((midi_channel == MIDI_CHANNEL_FILTER) || MIDI_OMNI) {

        // Handle notes
        if (((midi_status >> 4) == 0x8) || ((midi_status >> 4) == 0x9)) {

            limitNoteRange();

            if (((midi_status >> 4) == 0x9) && (midi_data2 > 0x0)) {
                handleNoteOn(midi_data1);
                handleVelocity(midi_data2);
            }

            if (((midi_status >> 4) == 0x8) ||
                (((midi_status >> 4) == 0x9) && (midi_data2 == 0x0))) {  // Note on with velocity 0 is note off
                handleNoteOff(midi_data1);
            }
        }


        // Handle control change
        if ((midi_status >> 4) == 0xB) {
            if (midi_data1 == RETRIG_MODE_CC) {
                handleRetrigModeToggle(midi_data2);
            } else if (CV2 == CC) {
                handleControlChange(midi_data1, midi_data2);
            }
        }


        // Handle pitch bend
        if ((CV2 == PITCH_BEND) && (midi_status >> 4) == 0xE) {
            handlePitchBend(midi_data1, midi_data2);
        }
    }
}



void parseMIDI(uint8_t midi_RX)
{
    // Nothing is done to the buffer when a RealTime Category message is received.
    if (midi_RX > 0xF7) return;

    // Buffer is cleared when a System Common Category Status (ie, 0xF0 to 0xF7) is received.
    if ((midi_RX > 0xEF) && (midi_RX < 0xF8)) {
        midi_status = 0x0;
        midi_channel = 0x0;
        midi_message_byte = 0;
        return;
    }

    // Buffer stores the status when a Voice Category Status (ie, 0x80 to 0xEF) is received.
    if ((midi_RX > 0x7F) && (midi_RX < 0xF0)) {
        midi_status = midi_RX;
        midi_channel = midi_status & 0x0F;
        midi_message_byte = 1;
        return;
    }

    // Any data bytes are ignored when the buffer is 0.
    if (midi_RX < 0x80) {
        if (!midi_status) return;

        if (midi_message_byte == 1) {
            midi_data1 = midi_RX;
            midi_message_byte = 2;

            if ((midi_status >> 4) == 0xC) handleProgramChange(midi_data1);  // handle program change

            return;
        }

        if (midi_message_byte == 2) {
            midi_data2 = midi_RX;
            midi_message_byte = 1;

            translateChannelMessage();

            return;
        }
    }
}



ISR (PCINT0_vect)
{
    if (!(PINB & 1 << PINB0)) {           // Ignore if DI is high
        GIMSK &= ~(1 << PCIE);            // Disable pin change interrupts
        TCCR0A = 2 << WGM00;              // CTC mode
        TCCR0B = 0 << WGM02 | 2 << CS00;  // Set prescaler to /8
        TCNT0 = 0;                        // Count up from 0
        OCR0A = 31;                       // Delay (31+1)*8 cycles
        TIFR |= 1 << OCF0A;               // Clear output compare flag
        TIMSK |= 1 << OCIE0A;             // Enable output compare interrupt
    }
}



ISR (TIMER0_COMPA_vect)
{
    TIMSK &= ~(1 << OCIE0A);  // Disable COMPA interrupt
    TCNT0 = 0;                // Count up from 0
    OCR0A = 63;               // Shift every (63+1)*8 cycles 32uS

    // Enable USI OVF interrupt, and select Timer0 compare match as USI Clock source:
    USICR = 1 << USIOIE | 0 << USIWM0 | 1 << USICS0;
    USISR = 1 << USIOIF | 8;  // Clear USI OVF flag, and set counter
}



ISR (USI_OVF_vect)
{
    uint8_t midi_RX;
    USICR = 0;           // Disable USI
    midi_RX = USIDR;
    GIFR = 1 << PCIF;    // Clear pin change interrupt flag.
    GIMSK |= 1 << PCIE;  // Enable pin change interrupts again

    // Wrong bit order so swap it:
    midi_RX = ((midi_RX >> 1) & 0x55) | ((midi_RX << 1) & 0xAA);
    midi_RX = ((midi_RX >> 2) & 0x33) | ((midi_RX << 2) & 0xCC);
    midi_RX = ((midi_RX >> 4) & 0x0F) | ((midi_RX << 4) & 0xF0);

    parseMIDI(midi_RX);
}



void loop()
{
    // do nothing, but wait for interrupts
}
