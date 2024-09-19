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


#include <Arduino.h>
// #include <avr/interrupt.h>
#include "midi.h"



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
