# MIDI, Media and Volume Controller USB Device

This is the source code of our MIDI, Media and Volume Controller USB Device based on the Arduino Pro Micro, made as a **MCE 202** lecture project at _Izmir University of Economics_.

![Device Example](https://github.com/user-attachments/assets/cdce6cdc-f0f2-4419-a1df-93067a962652)

## Requirements

- An Arduino board with native USB capabilities (PluggableUSB)
- Arduino IDE
- Components that are indicated in the next section
- Installing these libraries: 
    - [FastLED](https://docs.arduino.cc/libraries/fastled/) 
    - [MIDIUSB](https://docs.arduino.cc/libraries/midiusb) 
    - [HID-Project](https://docs.arduino.cc/libraries/hid-project/)

## Components

> Note: We have used INPUT_PULLUP pinMode for all inputs, keep that in mind while connecting the buttons to the circuit.

- A rotary encoder
    - CLK at **D2** pin *(needs to be an interrupt capable pin)*
    - DT at **D3** pin
    - SW at **D4** pin
- A switch at **D16** pin
- CRGB's *Din* at **D10** pin *(needs to be a PWM pin)*
- 6 buttons at the **D5**, **D6**, **D7**, **D8**, **D9** and **D14** pins

## Usage

The device has 2 modes: MIDI and MEDIA. Device mode can be changed by the switch.

### MEDIA

In MEDIA mode,

- The rotary encoder,
    - controlls operating system volume level, in normal conditions.
    - controlls brightness of the LEDs, if you press on top of it while rotating.
    - changes LED animation, if you double click on top of it.
- The buttons control the multimedia functions in the following order:
    - PLAY PREVIOUS
    - PLAY/PAUSE
    - PLAY NEXT
    - REWIND
    - MUTE
    - FAST FORWARD

### MIDI

In MIDI mode,

- The rotary encoder,
    - controlls 0th channel's 10th control of the Digital Audio Workstation. _(10th control is usually the modulation control in most DAWs)_
    - changes sensitivity of the control, if you press on top of it while rotating.
- The buttons sends note information with MIDI protocol, which can be assigned to different sounds in the DAW.
