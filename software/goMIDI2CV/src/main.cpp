// (*) All in the spirit of open-source and open-hardware
// Janost 2019 Sweden
// The goMIDI2CV interface
// http://blog.dspsynth.eu/diy-good-ol-midi-to-cv/
// Copyright 2019 DSP Synthesizers Sweden.
//
// Author: Jan Ostman; edited by Beau Sterling
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


volatile byte midi_state = 0;
volatile byte midi_running_status = 0;
volatile byte midi_note;
volatile byte midi_vel;

const byte MAX_NOTES = 16;
byte active_notes = 0;
byte note_buffer[MAX_NOTES] = {0};

byte high_note = 60;
byte low_note = 36;



void setup() {
    // Enable 64 MHz PLL and use as source for Timer1
    PLLCSR = 1 << PCKE | 1 << PLLE;

    // Setup the USI
    USICR = 0;                                                                          // Disable USI.
    GIFR = 1 << PCIF;                                                                   // Clear pin change interrupt flag.
    GIMSK |= 1 << PCIE;                                                                 // Enable pin change interrupts
    PCMSK |= 1 << PCINT0;                                                               // Enable pin change on pin 0

    // Set up Timer/Counter1 for PWM output
    TIMSK = 0;                                                                          // Timer interrupts OFF
    TCCR1 = 1 << PWM1A | 2 << COM1A0 | 1 << CS10;                                       // PWM A, clear on match, 1:1 prescale
    OCR1A = 0;                                                                          // Set initial Pitch to C2
    OCR1C = 239;                                                                        // Set count to semi tones

    GTCCR = 0;
    GTCCR = 1 << PWM1B | 2 << COM1B0;                                                   // PWM B, clear on match 

    // Setup GPIO
    pinMode(0, INPUT);                                                                  // Enable USI input pin
    pinMode(1, OUTPUT);                                                                 // Enable PWM output pin
    pinMode(2, OUTPUT);                                                                 // Enable Gate output pin
    pinMode(4, OUTPUT);                                                                 // Enable Pitchbend PWM output pin

    digitalWrite(2,LOW);                                                                // Set initial Gate to LOW;
}



void handleNoteOn() {
    for(byte n = 0; n < active_notes; n++) {
        if (note_buffer[n] == midi_note) {
            OCR1A = note_buffer[active_notes-1] << 2;
            return;
        }
    }
    note_buffer[active_notes] = midi_note;
    active_notes++;
    if (active_notes) OCR1A = note_buffer[active_notes-1] << 2;                         // Multiply note by 4 to set the voltage (1v/octave)
}


void handleNoteOff() {
    byte note_off_match = 0;
    for(byte n = 0; n < active_notes; n++) {
        if (note_buffer[n] == midi_note) {
            note_off_match = 1;
            if (n < (MAX_NOTES-1)) {                                                    // If end of buffer has not been reached, shift all notes to fill empty slot
                note_buffer[n] = note_buffer[n+1];
                note_buffer[n+1] = midi_note;
            }
        }
    }
    if (note_off_match) {
        note_off_match = 0;
        active_notes--;
        note_buffer[active_notes+1] = 0;
        if (active_notes) OCR1A = note_buffer[active_notes-1] << 2;                     // Multiply note by 4 to set the voltage (1v/octave)
    }
}


void sendGate() {
    if (active_notes > 0) {
        digitalWrite(2, HIGH);                                                          // Set Gate HIGH
    } else {
        if (active_notes > MAX_NOTES) active_notes = 0;
        digitalWrite(2, LOW);                                                           // Set Gate LOW
    }
}


void parseMIDI(byte midi_RX) {
    // Nothing is done to the buffer when a RealTime Category message is received.
    if (midi_RX > 0xF7) return;

    // Buffer is cleared when a System Common Category Status (ie, 0xF0 to 0xF7) is received.
    if ((midi_RX > 0xEF) && (midi_RX < 0xF8)) {
        midi_running_status = 0;
        midi_state = 0;
        return;
    }

    // Buffer stores the status when a Voice Category Status (ie, 0x80 to 0xEF) is received.
    if ((midi_RX > 0x7F) && (midi_RX < 0xF0)) {
        midi_running_status = midi_RX;
        midi_state = 1;
        return;
    }

    // Any data bytes are ignored when the buffer is 0.
    if (midi_RX < 0x80) {
        if (!midi_running_status) return;

        if (midi_state == 1) {
            midi_note = midi_RX;
            midi_state++;
            return;
        }

        if (midi_state == 2) {
            midi_vel = midi_RX;
            midi_state = 1;

            // Handle notes
            if ((midi_running_status == 0x80) || (midi_running_status == 0x90)) {
                if (midi_note < low_note) midi_note = low_note;                         // If note is lower than C2 set it to C2
                midi_note = midi_note - low_note;                                       // Subtract 36 to get into CV range
                if (midi_note > high_note) midi_note = high_note;                       // If note is higher than C7 set it to C7

                if ((midi_running_status == 0x90) && (midi_vel > 0x0)) {                // If note on
                    if (active_notes < MAX_NOTES) handleNoteOn();
                }

                if ((midi_running_status == 0x80) || 
                    ((midi_running_status == 0x90) && (midi_vel == 0x0))) {             // If note off
                    if (active_notes) handleNoteOff();
                }

                sendGate();
            }
            
            // Handle pitch bend
            if (midi_running_status == 0xE0) {
                if (midi_vel < 4) midi_vel = 4;                                         // Limit pitchbend to -60
                if (midi_vel > 119) midi_vel = 119;                                     // Limit pitchbend to +60
                midi_vel -= 4;                                                          // Center the pitchbend value
                OCR1B = midi_vel << 1;                                                  // Output pitchbend CV
            }
                        
            return;
        }
    }
}



ISR (PCINT0_vect) {
    if (!(PINB & 1 << PINB0)) {                                                         // Ignore if DI is high
        GIMSK &= ~(1 << PCIE);                                                          // Disable pin change interrupts
        TCCR0A = 2 << WGM00;                                                            // CTC mode
        TCCR0B = 0 << WGM02 | 2 << CS00;                                                // Set prescaler to /8
        TCNT0 = 0;                                                                      // Count up from 0
        OCR0A = 31;                                                                     // Delay (31+1)*8 cycles
        TIFR |= 1 << OCF0A;                                                             // Clear output compare flag
        TIMSK |= 1 << OCIE0A;                                                           // Enable output compare interrupt
    }
}


ISR (TIMER0_COMPA_vect) {
    TIMSK &= ~(1 << OCIE0A);                                                            // Disable COMPA interrupt
    TCNT0 = 0;                                                                          // Count up from 0
    OCR0A = 63;                                                                         // Shift every (63+1)*8 cycles 32uS

    // Enable USI OVF interrupt, and select Timer0 compare match as USI Clock source:
    USICR = 1 << USIOIE | 0 << USIWM0 | 1 << USICS0;
    USISR = 1 << USIOIF | 8;                                                            // Clear USI OVF flag, and set counter
}


ISR (USI_OVF_vect) {
    byte midi_RX;
    USICR = 0;                                                                          // Disable USI
    midi_RX = USIDR;
    GIFR = 1 << PCIF;                                                                   // Clear pin change interrupt flag.
    GIMSK |= 1 << PCIE;                                                                 // Enable pin change interrupts again

    // Wrong bit order so swap it:
    midi_RX = ((midi_RX >> 1) & 0x55) | ((midi_RX << 1) & 0xaa);
    midi_RX = ((midi_RX >> 2) & 0x33) | ((midi_RX << 2) & 0xcc);
    midi_RX = ((midi_RX >> 4) & 0x0f) | ((midi_RX << 4) & 0xf0);

    parseMIDI(midi_RX);
}



void loop() {

}
