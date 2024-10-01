# TinyMIDI2CV
ATTiny85 circuit which takes TTL-MIDI input and outputs a 1V/Octave CV and a Gate CV signal, as well as an attenuatable 0-5V CV signal (modes configurable via MIDI program change or SysEx messages).

This circuit can be embedded in other projects, or built standalone. I built mine in the form of a eurorack module.

<p float="left">
  <img src="./other%20resources/module_images/module_front.jpg" width=33% />
  <img src="./other%20resources/module_images/module_back.jpg" width=33% />
</p>


### To-Do:
- EEPROM write cv2 mode configurations etc. after changes
- Update hardware drawings
- Design compact PCB
- (include ISP pins?)

### Changes
- Added SysEx listener for configuring CV2, MIDI Channel, retrig mode, CC filter. Added Paraphonic Note mode for CV2. Added more options for retrigger. Updated project name. Created SysEx reference document.

- Further improvements to note buffer / gate behavior. Added MIDI CC legato (68) handler to enable/disable gate retrigger. MIDI filter default channel is now 1. More function compartmentalization for readability. Added alternative methods to control CV2 output, selectable via MIDI Program Change. More CV2 functions to come soon.

- Original code (in goMIDI2CV.ino file) has been moved to jo_original branch to simplify the repo

- MIDI channel filtering is fixed in latest commmit, and default is channel 16. Can change channel or enable Omni in source and recompile if desired.

- Updated the [schematic](./hardware/TinyMIDI2CV.pdf), which should now be suitable for creating a small Eurorack module.  It's only my second module I have built, so feel free to improve it.

-  Overhauled most of the code styling, and fixed the Gate CV behavior. Now Gate remains High as long as any note is still held.  Also, retrigger functionality was added. Gate pin goes Low very briefly, then quickly back to High, any time a new note is played while other notes are held.

- A note buffer array now remembers active notes in the order they were played.  Note CV output returns to previous note value if it is still held when the most recent note is released.  Note CV output does not change when the final note is released, but the Gate CV goes Low.

- Fixed the way the code responds to MIDI messages and handles running status. Pitch bend is now functional.  Broke out handlers for different types of messages like note on/off, bend, gate, etc. into their own defined functions.

- After much fiddling with different summing circuits and testing with my synth, I determined that a seperate output jack for pitchbend is more convenient for me than coupling it to the Note CV.  This is shown in the new schematic. If you find a nicer way to do it, let me know.

- The barest bones for handling MIDI CC and velocity, and sending CV Trigger output are in place as well. Further development on those will be started soon in another branch.

### Programming
I program my ATTiny85 using an Arudino Uno as an ISP programmer, following instructions shown here and other places:
[https://circuitdigest.com/microcontroller-projects/programming-attiny85-microcontroller-ic-using-arduino]

You can paste the contents of [main.cpp](./software/src/main.cpp) into the ArduinoIDE and directly follow the instructions on that site to load it to your chip.
Or if you prefer VSCode with PlatformIO, you can use the settings in this repo's [platform.ini](./software/platform.ini) file to program the ATTiny85 using an Uno as an ISP. You may need to change some of those parameters if you use a different programmer.

Notice, that site doesn't explain how to set the fuse bits before flashing the firmware, but I included a [text file](./software/attiny85%20fuse%20settings.txt) with brief instructions on this, and the relevant terminal commands (for Windows).



## DIY Good Ol’ MIDI to CV
This project began as a reworked and expanded version of the DIY Good Ol’ MIDI to CV project by Jan Ostman.
Original project: [https://www.hackster.io/janost/diy-good-ol-midi-to-cv-d0e2bf]
