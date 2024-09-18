/*
    TinyMIDI2CV

    Copyright 2023-2024 Beau Sterling (Aether Soundlab)

    Based on DIY Good Ol’ MIDI to CV by Jan Ostman:
        (*) All in the spirit of open-source and open-hardware
        Janost 2019 Sweden
        The goMIDI2CV interface
        http://blog.dspsynth.eu/diy-good-ol-midi-to-cv/
        Copyright 2019 DSP Synthesizers Sweden.


    This program is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    This program is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
    more details.

    You should have received a copy of the GNU General Public License along with
    This program. If not, see <https://www.gnu.org/licenses/>.
*/



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
const uint8_t MIDI_CHANNEL_FILTER = 0x0;            // MIDI channel 1 (zero-indexed, 0x0-0xF)
volatile uint8_t midi_cc_filter = 3;


const uint8_t POLYPHONY_MAX = 16;                   // Set buffer and gate "polyphony" limit
volatile uint8_t note_buffer[POLYPHONY_MAX] = {0};
volatile uint8_t active_notes = 0;                  // Gate will be open while any keys are pressed (any notes active)

const uint8_t LOW_NOTE = 36;                        // Any note lower than C2 will be interpreted as C2
const uint8_t HIGH_NOTE = 96;                       // Any note higher than C7 will be interpreted as C7

enum PARAPHONIC_PRIORITY { PARA_LAST, PARA_LO, PARA_HI };
PARAPHONIC_PRIORITY para_priority = PARA_LAST;

/* unused, delete in future?:
const int BEND_MAX = 0x3FFF;
const int BEND_CENTER = 0x2000;
const int BEND_MIN = 0x0;
*/

enum CV2_MODE {
    PITCH_BEND, MODWHEEL, CC, VELOCITY,
    NOTE, INV_NOTE, GATE, INV_GATE,
    TRIG, ENVELOPE, LFO, SYNC,
    PARAPHONIC, RT_TOGGLE, UNUSED1, UNUSED2
};
volatile CV2_MODE cv2 = MODWHEEL;


const uint8_t RETRIG_MODE_CC = 68;
enum GATE_STATE { CLOSED, OPEN, RETRIG };

enum RETRIG_MODE {
    RT_OFF,    // never retrigger
    RT_NEW,    // retrigger when new notes are played
    RT_ALWAYS  // also retrigger when notes are released if other notes are still held
}; RETRIG_MODE retrigger = RT_NEW;


#define SYSEX_ID 0x7D
#define DEVICE_ID 0x42

enum SYSEX_BYTE_ID {
    START, SYXID, DEVID, PADDING,
    COMMAND, VALUE, CHECKSUM, STOP
};

const uint8_t SYSEX_BUFFER_SIZE = 16;
volatile uint8_t sysex_buffer[SYSEX_BUFFER_SIZE];
volatile uint8_t sysex_byte_count = 0;

volatile bool sysex_listen = false;
volatile bool sysex_ignore = false;
volatile bool sysex_msg_ok = false;



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



void setCV2Mode(uint8_t mode) {
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

            case CV2_MODE::RT_TOGGLE:
                switch(retrigger) {
                    case RETRIG_MODE::RT_NEW:
                        retrigger = RT_ALWAYS;
                        break;
                    case RETRIG_MODE::RT_ALWAYS:
                        retrigger = RT_OFF;
                        break;
                    case RETRIG_MODE::RT_OFF:
                    default:
                        retrigger = RT_NEW;
                        break;
                }
                break;

            default:  // CV2_MODE::MODWHEEL
                cv2 = CC;
                midi_cc_filter = 1;
                break;
        }
}



void handleProgramChange(uint8_t program)
{
    if ((midi_channel == MIDI_CHANNEL_FILTER) || MIDI_OMNI) {
        setCV2Mode(program);
    }
}



void startSysExListener()
{
    sysex_listen = true;
}



void handleSysEx(uint8_t syx)
{
    // check for buffer overflow
    if (sysex_byte_count >= SYSEX_BUFFER_SIZE) {
        for (int i = 0; i < SYSEX_BUFFER_SIZE; ++i) {
            sysex_buffer[i] = 0x0;
        }
        sysex_byte_count = 0;
        sysex_ignore = true;
        return;  // reset entire buffer and ignore
    }


    // load buffer with new byte
    sysex_buffer[sysex_byte_count] = syx;
    sysex_byte_count++;


    // check to see if message is valid
    if (!sysex_msg_ok) {
        if (sysex_buffer[0] == 0xF0) {  // if valid sysex start byte
            if (sysex_byte_count >= 3) {
                if (sysex_buffer[1] == SYSEX_ID || sysex_buffer[2] == DEVICE_ID) {  // if valid address
                    sysex_msg_ok = true;
                    return;  // message ok, load the rest
                } else {
                    for (int i = 0; i < sysex_byte_count; ++i) {
                        sysex_buffer[i] = 0x0;
                    }
                    sysex_byte_count = 0;
                    sysex_ignore = true;
                    return;  // invalid address, reset and ignore buffer
                }
            } else { return; }  // not enough bytes yet
        } else {
            sysex_buffer[0] = 0x0;
            sysex_byte_count = 0;
            sysex_ignore = true;
            return;  // invalid start byte, reset and ignore buffer
        }
    }


    if (syx == 0x7F) { // if current byte is end of message flag
        uint8_t value;
        if (sysex_buffer[SYSEX_BYTE_ID::COMMAND] == 0x02) {
            value = sysex_buffer[SYSEX_BYTE_ID::VALUE];
            setCV2Mode(value);
            return;
        }
        if (sysex_buffer[SYSEX_BYTE_ID::COMMAND] == 0x03) {
            value = sysex_buffer[SYSEX_BYTE_ID::VALUE];
            midi_cc_filter = value;
            return;
        }
    }

    // example message:
    // F0 7D 42 00 02 05 00 7F
    // means set CV2 mode to INV_NOTE
}



void stopSysExListener(uint8_t syx)
{
    handleSysEx(syx);
    for (int i = 0; i < SYSEX_BUFFER_SIZE; ++i) {
        sysex_buffer[i] = 0x0;
    }
    sysex_byte_count = 0;
    sysex_listen = false;
    sysex_ignore = false;
    sysex_msg_ok = false;
}



void limitNoteRange()
{
    if (midi_data1 < LOW_NOTE) midi_data1 = LOW_NOTE;    // If note is lower than C2 set it to C2
    if (midi_data1 > HIGH_NOTE) midi_data1 = HIGH_NOTE;  // If note is higher than C7 set it to C7
    midi_data1 -= LOW_NOTE;                              // Subtract 36 to get into CV range
}



void sendInvNote(uint8_t note)
{
    OCR1B = OCR1C - (note << 2);
}



void sendNote(uint8_t note)
{
    OCR1A = note << 2;  // Multiply note by 4 to set the voltage (1v/octave)
    if (cv2 == NOTE) OCR1B = note << 2;
    if (cv2 == INV_NOTE) sendInvNote(note);
}


void sendParaNote(uint8_t note)
{
    OCR1B = note << 2;
}

void handleRetrigModeToggle(uint8_t cc_value)
{
    if (cc_value > 83)
        retrigger = RT_ALWAYS;
    else if (cc_value > 42 && cc_value < 84)
        retrigger = RT_NEW;
    else
        retrigger = RT_OFF;
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



void sendGate(GATE_STATE gate_state)
{
    switch (gate_state) {
        case GATE_STATE::OPEN:
            digitalWrite(GATE_CV_PIN, HIGH);  // Set Gate HIGH
            sendGateCV(HIGH);
            break;

        case GATE_STATE::RETRIG:
            digitalWrite(GATE_CV_PIN, LOW);  // Set Gate LOW
            sendGateCV(LOW);
            delayMicroseconds(32);
            digitalWrite(GATE_CV_PIN, HIGH);
            sendGateCV(HIGH);
            break;

        case GATE_STATE::CLOSED:
        default:
            digitalWrite(GATE_CV_PIN, LOW);
            sendGateCV(LOW);
            break;
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

    OCR1B = bend_CV;  // Output pitchbend CV
}



void handleParaPriority(uint8_t note, uint8_t last_note)
{
    switch (para_priority) {
        case PARA_LO:
            if (note > last_note) {
                sendNote(note);
                sendParaNote(last_note);
            } else {
                sendNote(last_note);
                sendParaNote(note);
            }
            break;

        case PARA_HI:
            if (note > last_note) {
                sendNote(last_note);
                sendParaNote(note);
            } else {
                sendNote(note);
                sendParaNote(last_note);
            }
            break;

        case PARA_LAST:
        default:
            sendNote(note);
            sendParaNote(last_note);
            break;
    }
}



void handleNoteOn(uint8_t note)
{
    for (uint8_t n = 0; n < active_notes; n++) {  // If note is already in the buffer, play it, but don't add it again
        if (note_buffer[n] == note) {
            sendNote(note);  // handle paraphonic?
            return;
        }
    }

    if (active_notes < POLYPHONY_MAX) {
        note_buffer[active_notes] = note;
        active_notes++;

        if (cv2 == PARAPHONIC) {
            if (active_notes > 1) {
                uint8_t last_note = note_buffer[active_notes-2];
                handleParaPriority(note, last_note);
            } else {
                sendNote(note);
                sendParaNote(note);
            }
        } else {
            sendNote(note);
        }

        if (retrigger != RT_OFF)
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
        note_buffer[active_notes] = 0;  // Remove requested note and play next in buffer
        active_notes--;

        if (active_notes) {
            note = note_buffer[active_notes-1];
            if (cv2 == PARAPHONIC) {
                if (active_notes > 1) {
                    uint8_t last_note = note_buffer[active_notes-2];
                    handleParaPriority(note, last_note);
                } else {
                    sendNote(note);
                    sendNote(note);
                }
            } else {
                sendNote(note);
            }

            if (retrigger == RT_ALWAYS)
                sendGate(GATE_STATE::RETRIG);
            else
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
            } else if (cv2 == CC) {
                handleControlChange(midi_data1, midi_data2);
            }
        }


        // Handle pitch bend
        if ((cv2 == PITCH_BEND) && (midi_status >> 4) == 0xE) {
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

        if (midi_RX == 0xF0) startSysExListener();
        if (midi_RX == 0xF7) stopSysExListener(midi_RX);
        if (sysex_listen && !sysex_ignore) handleSysEx(midi_RX);

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
