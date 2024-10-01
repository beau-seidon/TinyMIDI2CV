
## SET MIDI CHANNEL
#### COMMAND = 01, DATA = [00-10]

- F0 7D 42 01 00 F7 = Channel 01
- F0 7D 42 01 01 F7 = Channel 02
- F0 7D 42 01 02 F7 = Channel 03
- . . .
- F0 7D 42 01 0E F7 = Channel 15
- F0 7D 42 01 0F F7 = Channel 16
- F0 7D 42 01 10 F7 = Omni ON (^others set Omni to OFF)


## SET MIDI CC FILTER
#### COMMAND = 02, DATA = [00-FF]

- F0 7D 42 02 00 F7 = CC# 1
- F0 7D 42 02 01 F7 = CC# 2
- F0 7D 42 02 02 F7 = CC# 3
- . . .
- F0 7D 42 02 FE F7 = CC# 127
- F0 7D 42 02 FF F7 = CC# 128


## SET CV2 SOURCE
#### COMMAND = 03, DATA = [00-FF]

- F0 7D 42 03 00 F7 - Pitch Bend
- F0 7D 42 03 01 F7 - Mod Wheel
- F0 7D 42 03 02 F7 - CC
- F0 7D 42 03 03 F7 - Velocity
- F0 7D 42 03 04 F7 - Note
- F0 7D 42 03 05 F7 - Note (Inverted Keyboard)
- F0 7D 42 03 06 F7 - Gate
- F0 7D 42 03 07 F7 - Gate (Inverted)
- F0 7D 42 03 08 F7 - Paraphonic Notes
- F0 7D 42 03 09 F7 - Increment Paraphonic Mode
- F0 7D 42 03 0A F7 - Increment Retrigger Mode
- F0 7D 42 03 0B F7 - Unused...
- F0 7D 42 03 0C F7 - Trigger (Not implemented yet)
- F0 7D 42 03 0D F7 - Envelope (Not implemented yet)
- F0 7D 42 03 0E F7 - LFO (Not implemented yet)
- F0 7D 42 03 0F F7 - Sync (Not implemented yet)


## SET PARAPHONIC MODE
#### COMMAND = 04, DATA = [00-08]

- F0 7D 42 04 00 F7 - Recent (CV1 = new)
- F0 7D 42 04 01 F7 - Recent Inverted (CV2 = new)
- F0 7D 42 04 02 F7 - Recent Lo (CV2 = low)
- F0 7D 42 04 03 F7 - Recent Hi (CV2 = high)
- F0 7D 42 04 04 F7 - Outer (CV1 = high)
- F0 7D 42 04 05 F7 - Outer Inverted (CV1 = low)
- F0 7D 42 04 06 F7 - Low (CV2 = low)
- F0 7D 42 04 07 F7 - Hi (CV2 = high)
- F0 7D 42 04 08 F7 - Pedal (CV2 = first note in buffer)


## SET RETRIGGER MODE
#### COMMAND = 05, DATA = [00-02]

- F0 7D 42 05 00 F7 - Off
- F0 7D 42 05 01 F7 - New
- F0 7D 42 05 02 F7 - Always
