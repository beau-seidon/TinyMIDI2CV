// (*) All in the spirit of open-source and open-hardware 
// Janost 2019 Sweden  
// The goMIDI2CV interface
// http://blog.dspsynth.eu/diy-good-ol-midi-to-cv/
// Copyright 2019 DSP Synthesizers Sweden. 
// 
// Author: Jan Ostman 
// 
// This program is free software: you can redistribute it and/or modify 
//it under the terms of the GNU General Public License as published by 
// the Free Software Foundation, either version 3 of the License, or 
// (at your option) any later version. 
// This program is distributed in the hope that it will be useful, 
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
// GNU General Public License for more details.
  
//Set Fuses to E1 DD FE for PLLCLK 16MHz
  
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
  
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
  
volatile uint8_t MIDISTATE=0;
volatile uint8_t MIDIRUNNINGSTATUS=0;
volatile uint8_t MIDINOTE;
volatile uint8_t MIDIVEL;
  
void setup() {
 // Enable 64 MHz PLL and use as source for Timer1
 PLLCSR = 1<<PCKE | 1<<PLLE;     
 // Set up Timer/Counter1 for PWM output
 TIMSK = 0;                     // Timer interrupts OFF
 TCCR1 = 1<<PWM1A | 2<<COM1A0 | 1<<CS10; // PWM A, clear on match, 1:1 prescale
 // Setup GPIO
 pinMode(1, OUTPUT); // Enable PWM output pin
 pinMode(0, INPUT);  // Enable USI input pin
 pinMode(2, OUTPUT); // Enable Gate output pin
 pinMode(4,OUTPUT);  // Enable Pitchbend PWM output pin
 //Setup the USI
 USICR = 0;          // Disable USI.
 GIFR = 1<<PCIF;     // Clear pin change interrupt flag.
 GIMSK |= 1<<PCIE;   // Enable pin change interrupts
 PCMSK |= 1<<PCINT0; // Enable pin change on pin 0
 GTCCR = 0;
 OCR1C = 239; //Set count to semi tones
 OCR1A = 0; //Set initial Pitch to C2
 digitalWrite(2,LOW); //Set initial Gate to LOW;
 pinMode(2, OUTPUT); // Enable PB4 output pin for pitchbend CV
 GTCCR = 1<<PWM1B | 2<<COM1B0; // PWM B, clear on match
}
  
ISR (PCINT0_vect) {
 if (!(PINB & 1<<PINB0)) {       // Ignore if DI is high
   GIMSK &= ~(1<<PCIE);          // Disable pin change interrupts
   TCCR0A = 2<<WGM00;            // CTC mode
   TCCR0B = 0<<WGM02 | 2<<CS00;  // Set prescaler to /8
   TCNT0 = 0;                    // Count up from 0
   OCR0A = 31;                   // Delay (31+1)*8 cycles
   TIFR |= 1<<OCF0A;             // Clear output compare flag
   TIMSK |= 1<<OCIE0A;           // Enable output compare interrupt
 }
}
  
ISR (TIMER0_COMPA_vect) {
 TIMSK &= ~(1<<OCIE0A);          // Disable COMPA interrupt
 TCNT0 = 0;                      // Count up from 0
 OCR0A = 63;                    // Shift every (63+1)*8 cycles 32uS
 // Enable USI OVF interrupt, and select Timer0 compare match as USI Clock source:
 USICR = 1<<USIOIE | 0<<USIWM0 | 1<<USICS0;
 USISR = 1<<USIOIF | 8;          // Clear USI OVF flag, and set counter
}
  
ISR (USI_OVF_vect) {
 uint8_t MIDIRX;
 USICR = 0;                      // Disable USI         
 MIDIRX = USIDR;
 GIFR = 1<<PCIF;                 // Clear pin change interrupt flag.
 GIMSK |= 1<<PCIE; // Enable pin change interrupts again
  
 //Wrong bit order so swap it
 MIDIRX = ((MIDIRX >> 1) & 0x55) | ((MIDIRX  << 1) & 0xaa);
 MIDIRX = ((MIDIRX >> 2) & 0x33) | ((MIDIRX  << 2) & 0xcc);
 MIDIRX = ((MIDIRX >> 4) & 0x0f) | ((MIDIRX  << 4) & 0xf0);
  
 //Parse MIDI data
 if ((MIDIRX>0xBF)&&(MIDIRX<0xF8)) {
   MIDIRUNNINGSTATUS=0;
   MIDISTATE=0;
   return;
 }
 if (MIDIRX>0xF7) return;
 if (MIDIRX & 0x80) {
   MIDIRUNNINGSTATUS=MIDIRX;
   MIDISTATE=1;
   return;
 }
 if (MIDIRX < 0x80) {
   if (!MIDIRUNNINGSTATUS) return;
   if (MIDISTATE==1) {
   MIDINOTE=MIDIRX;
   MIDISTATE++;
   return;
 }
 if (MIDISTATE==2) {
   MIDIVEL=MIDIRX;
   MIDISTATE=1;
   if ((MIDIRUNNINGSTATUS==0x80)||(MIDIRUNNINGSTATUS==0x90)) {
     if (MIDINOTE<36) MIDINOTE=36; //If note is lower than C2 set it to C2
     MIDINOTE=MIDINOTE-36; //Subtract 36 to get into CV range
     if (MIDINOTE>60) MIDINOTE=60; //If note is higher than C7 set it to C7
       if (MIDIRUNNINGSTATUS == 0x90) { //If note on
       if (MIDIVEL>0) digitalWrite(2, HIGH); //Set Gate HIGH
       if (MIDIVEL==0) digitalWrite(2, LOW); //Set Gate HIGH
       OCR1A = MIDINOTE<<2; //Multiply note by 4 to set the voltage (1v/octave)
     }
     if (MIDIRUNNINGSTATUS == 0x80) { //If note off
       digitalWrite(2, LOW); //Set Gate LOW
       OCR1A = MIDINOTE<<2; //Multiply note by 4 to set the voltage (1v/octave)
     }
    if (MIDIRUNNINGSTATUS == 0xE0) { //If pitchbend data
      if (MIDIVEL<4) MIDIVEL=4; //Limit pitchbend to -60
      if (MIDIVEL>119) MIDIVEL=119; //Limit pitchbend to +60
      MIDIVEL-=4; //Center the pitchbend value
      OCR1B = (MIDIVEL<<1); //Output pitchbend CV
}
   }
   return;
 }
}
  
void loop() {
  
}
 