# goMIDI2CV
Largely reworked version of the goMIDI2CV project by Jan Ostman. 


Original project: https://www.hackster.io/janost/diy-good-ol-midi-to-cv-d0e2bf

DIY Good Ol’ MIDI to CV
  The goMIDI2CV takes TTL-MIDI input and outputs a 1V/Octave CV and a gate signal.

    <img1>

Story
  This is really a tutorial on how to use the USI component on the ATtiny as a UART-RX and MIDI is as good as any application for a UART.
  
  There are 3 additional unused outputs on the ATtiny45/85 that can be used for CV/Gates/Triggers or Sync. If you run this code on the ATtiny44/84 there is a total of 10 outputs that can be used.
  
  What if you don’t like these fancy USB-MIDI converters or plain have a requirement to convert your MIDI keyboard to a CV/gate signal for some vintage gear?
  
  Now you can.
  
  The goMIDI2CV takes a TTL-MIDI input and outputs a 1V/Octave CV and a gate signal.

    <img2>

  The code runs on a ATtiny45/85 microcontroller.
  
  PB0 is the MIDI input.PB1 is the CV output.PB2 is the gate output
  
  The MIDI-in is a TTL input and needs the usual optocoupler.
  
    <img3>

  The CV-out is a PWM signal and needs a lowpass filter. Range is C2-C7, 0-5 volts.

  The gate output is 5 volt for note-on and 0 volt for Note-off.
  
  Note: The chip can run off MIDI ghost-power but for the CV to play in tune the supply voltage needs to be exactly 5 volts.

Building blocks
  There are 2 major basic components needed for making the converter:
  
    A D/A converter for the CV output
    A serial UART input for the MIDI
  
  The ATtiny does not have a DAC so we create one using PWM.
  
  And its also lacking a UART but we can use the USI component for that one.

    <code1>

  First we need to setup the chip. This code takes care of setting up the PWM and USI.
  
  The fuses are important as we want to run the Tiny on a internal 16MHz clock.

    <code2>

  Next we need a pinchange interrupt to handle the start bit in the MIDI serial input.

  MIDI is serial data at 31250bits/s so one bit time is 32 microseconds.

    <img4>
  
  The start of a byte causes a pin-change interrupt. In the pin-change interrupt service routine we check that it’s a falling edge, and if so we set up Timer/Counter0 in CTC mode. We want to set up a delay of half a bit, to get into the middle of the start bit. The closest we can get to that is a prescaler of 8 and a compare match of 32. Finally we clear and enable the output compare interrupt.

    <code3>

  The compare match interrupt occurs in the middle of the start bit. In the compare match interrupt service routine we reset the compare match to the duration of one bit, 64 (32uS), enable the USI to start shifting in the data bits on the next compare match, and enable the USI overflow interrupt.

    <code4>

  Note that we set the Wire Mode to 0 with 0<<USIWM0. This ensures that the output of the USI shift register won’t affect the CV output pin, PB1.
  
  When 8 bits have been shifted in the USI overflow interrupt occurs. The interrupt service routine disables the USI, reads the USI shift register, and enables the pin change interrupt ready for the next byte.    

    <code5>

  The UART sends the bits LSB first, whereas the USI assumes that the MSB is first, so we need to reverse the order of the bits after reception.
  
  The MIDI parser is also included here.
  
  Running status is what 99% of all MIDI synths and sequencers do today and it means that if the status byte is the same as the previous one its not transmitted which results in less bytes on the line.
  
  Handling “Running status”
    1. Buffer is cleared (ie, set to 0) at power up.
    2. Buffer stores the status when a Voice Category Status (ie, 0x80 to 0xEF) is received.
    3. Buffer is cleared when a System Common Category Status (ie, 0xF0 to 0xF7) is received.
    4. Nothing is done to the buffer when a RealTime Category message is received.
    5. Any data bytes are ignored when the buffer is 0.
  
  The parser handles running status and processes the noteon/noteoff and CV.
  
  The 0x80, 0x90 and 0xB0 bytes here is MIDI status for Note-on, Note-off and MIDI-CC at channel #01. If you want some other receive channel, say channel #02, you change the lower nibble to 0x01, i.e 0x91.
  
  Also a majority of transmitters use Note-on with zero velocity for Note-off.
  
  The CV output is in the range C2 to C7 (0-5 volt) so we subtract 36 from the key to make C2 0 volts.
  
  We also check if the note is above C7 which is 5 volts and limit it.
  
  Then we write the key to the CV output.
  
  8-bits is actually a too low resolution for the pitch but a trick makes it work.
  The CV range is set to 0-239 instead of 255 and that makes each seminote exactly 4 steps.
  
  If the event is a note-on we set the Gate output high and a note-off sets it low.

Adding a pitchbend CV
  The 240 note CV levels are too few steps to add pitchbend values, it will be noticeably steppy.
  
  So we use another PWM output and CV that outputs the pitchbend value. That CV can be used separately or add it to the note CV through a 4.7K resistor.
  
  Add this code to the setup and MIDI parser:
  
    <code6>

  Here is the full code:

    <code7>
