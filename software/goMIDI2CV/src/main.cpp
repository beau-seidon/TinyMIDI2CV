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


volatile uint8_t midi_message_byte = 0;
volatile byte midi_status = 0x0;
volatile byte midi_channel = 0x0;
volatile byte midi_data1 = 0x0;
volatile byte midi_data2 = 0x0;

const bool MIDI_OMNI = true;                                                              // Set true to ignore filter, or false to use a single midi channel
const byte MIDI_CHANNEL_FILTER = 0x15;                                                    // MIDI channel 16

const uint8_t MAX_NOTES = 16;                                                             // Set buffer and gate "polyphony" limit
byte note_buffer[MAX_NOTES] = {0};
uint8_t active_notes = 0;                                                                 // Gate will be open while any keys are pressed (notes active)



void setup() {
    // Enable 64 MHz PLL and use as source for Timer1
    PLLCSR = 1 << PCKE | 1 << PLLE;

    // Setup the USI
    USICR = 0;                                                                            // Disable USI.
    GIFR = 1 << PCIF;                                                                     // Clear pin change interrupt flag.
    GIMSK |= 1 << PCIE;                                                                   // Enable pin change interrupts
    PCMSK |= 1 << PCINT0;                                                                 // Enable pin change on pin 0

    // Set up Timer/Counter1 for PWM output
    TIMSK = 0;                                                                            // Timer interrupts OFF
    TCCR1 = 1 << PWM1A | 2 << COM1A0 | 1 << CS10;                                         // PWM A, clear on match, 1:1 prescale
    OCR1A = 0;                                                                            // Set initial Pitch to C2
    OCR1C = 239;                                                                          // Set count to semi tones

    GTCCR = 0;
    GTCCR = 1 << PWM1B | 2 << COM1B0;                                                     // PWM B, clear on match 

    // Setup GPIO
    pinMode(0, INPUT);                                                                    // Enable USI input pin
    pinMode(1, OUTPUT);                                                                   // Enable PWM output pin
    pinMode(2, OUTPUT);                                                                   // Enable Gate output pin
    pinMode(4, OUTPUT);                                                                   // Enable Pitchbend PWM output pin

    digitalWrite(2,LOW);                                                                  // Set initial Gate to LOW;
}



void limitNoteRange() {
    if (midi_data1 < 36) midi_data1 = 36;                                                 // If note is lower than C2 set it to C2
    midi_data1 = midi_data1 - 36;                                                         // Subtract 36 to get into CV range
    if (midi_data1 > 60) midi_data1 = 60;                                                 // If note is higher than C7 set it to C7
}


void handleNoteOn(byte note) {
    for(byte n = 0; n < active_notes; n++) {                                              // If note is already in the buffer, play it, but don't add it again
        if (note_buffer[n] == note) {
            OCR1A = note_buffer[active_notes-1] << 2;
            return;
        }
    }
    note_buffer[active_notes] = note;
    active_notes++;
    if (active_notes) OCR1A = note_buffer[active_notes-1] << 2;                           // Multiply note by 4 to set the voltage (1v/octave)
}


void handleNoteOff(byte note) {
    bool note_off_match = false;
    for(byte n = 0; n < active_notes; n++) {                                              // Check buffer to see if note is active
        if (note_buffer[n] == note) {
            note_off_match = true;
            if (n < (MAX_NOTES-1)) {                                                      // If note is removed from middle of buffer, shift all notes to prevent empty slots
                note_buffer[n] = note_buffer[n+1];
                note_buffer[n+1] = note;
            }
        }
    }
    if (note_off_match) {
        note_off_match = false;
        active_notes--;
        note_buffer[active_notes+1] = 0;                                                  // Remove requested note and play next in buffer
        if (active_notes) OCR1A = note_buffer[active_notes-1] << 2;                       // Multiply note by 4 to set the voltage (1v/octave)
    }
}


void sendGate() {
    if (active_notes > 0) {
        digitalWrite(2, HIGH);                                                            // Set Gate HIGH
    } else {
        if (active_notes > MAX_NOTES) active_notes = 0;
        digitalWrite(2, LOW);                                                             // Set Gate LOW
    }
}


void parseMIDI(byte midi_RX) {
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
            return;
        }

        if (midi_message_byte == 2) {
            midi_data2 = midi_RX;
            midi_message_byte = 1;

            if ((midi_channel == MIDI_CHANNEL_FILTER) || MIDI_OMNI) {
                // Handle notes
                if ((midi_status & 0x80) || (midi_status & 0x90)) {
                    limitNoteRange();

                    if ((midi_status & 0x90) && (midi_data2 > 0x0)) {                     // If note on
                        if (active_notes < MAX_NOTES) handleNoteOn(midi_data1);
                    }

                    if ((midi_status & 0x80) || 
                        ((midi_status & 0x90) && (midi_data2 == 0x0))) {                  // If note off
                        if (active_notes) handleNoteOff(midi_data1);
                    }

                    sendGate();
                }

                // Handle control change
                if (midi_status & 0xB0) {
                    // for future use
                }
                
                // Handle pitch bend
                if (midi_status & 0xE0) {
                    if (midi_data2 < 4) midi_data2 = 4;                                   // Limit pitchbend to -60
                    if (midi_data2 > 119) midi_data2 = 119;                               // Limit pitchbend to +60
                    midi_data2 -= 4;                                                      // Center the pitchbend value
                    OCR1B = midi_data2 << 1;                                              // Output pitchbend CV
                }
            }
            return;
        }
    }
}



ISR (PCINT0_vect) {
    if (!(PINB & 1 << PINB0)) {                                                           // Ignore if DI is high
        GIMSK &= ~(1 << PCIE);                                                            // Disable pin change interrupts
        TCCR0A = 2 << WGM00;                                                              // CTC mode
        TCCR0B = 0 << WGM02 | 2 << CS00;                                                  // Set prescaler to /8
        TCNT0 = 0;                                                                        // Count up from 0
        OCR0A = 31;                                                                       // Delay (31+1)*8 cycles
        TIFR |= 1 << OCF0A;                                                               // Clear output compare flag
        TIMSK |= 1 << OCIE0A;                                                             // Enable output compare interrupt
    }
}


ISR (TIMER0_COMPA_vect) {
    TIMSK &= ~(1 << OCIE0A);                                                              // Disable COMPA interrupt
    TCNT0 = 0;                                                                            // Count up from 0
    OCR0A = 63;                                                                           // Shift every (63+1)*8 cycles 32uS

    // Enable USI OVF interrupt, and select Timer0 compare match as USI Clock source:
    USICR = 1 << USIOIE | 0 << USIWM0 | 1 << USICS0;
    USISR = 1 << USIOIF | 8;                                                              // Clear USI OVF flag, and set counter
}


ISR (USI_OVF_vect) {
    byte midi_RX;
    USICR = 0;                                                                            // Disable USI
    midi_RX = USIDR;
    GIFR = 1 << PCIF;                                                                     // Clear pin change interrupt flag.
    GIMSK |= 1 << PCIE;                                                                   // Enable pin change interrupts again

    // Wrong bit order so swap it:
    midi_RX = ((midi_RX >> 1) & 0x55) | ((midi_RX << 1) & 0xaa);
    midi_RX = ((midi_RX >> 2) & 0x33) | ((midi_RX << 2) & 0xcc);
    midi_RX = ((midi_RX >> 4) & 0x0f) | ((midi_RX << 4) & 0xf0);

    parseMIDI(midi_RX);
}



void loop() {
    // do nothing, but wait for interrupts
}
