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



// Set Fuses to E1 DD FE for PLLCLK 16MHz



#include <Arduino.h>
// #include <avr/io.h>
// #include <avr/pgmspace.h>
#include "global.h"
#include "midi.h"
#include "gate.h"
#include "cv2.h"



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
    sendGate(CLOSED);
    setCV2Mode(MODWHEEL);
}



void loop()
{
    // do nothing, everything happens via interrupts
}
