# RC6 remote control translator for PlayStation 2
With this project you can program an ATtiny13 microcontroller to translate signals from a remote control using RC6 protocol (used by modern Philips TVs) into the SIRC protocol used by the PS2.

This is a very specific use case so I doubt anyone will use it like that. If you just want to use your PS2 and TV with the same remote, just get a universal remote control that supports both RC6 and SIRC protocols. But if you're a programming nerd like me, knock yourself out :)

**If you're looking for an implementation of RC6 reception on an ATTiny13, take a look!** You might need it for something completely different, but it will work nicely.

## How it works
First you have to get an ATtiny13 microcontroller and program it with this software here. See the Configuration section to see how you can make it fit your needs.

Then you have to open up your PS2, disconnect the IR receiver from the microcontroller (by desoldering a resistor or scratching off a trace, depending on your mainboard model). You can use the IR receiver signal that's on the mainboard already - the receiver can pick up and demodulate an RC6 signal just fine.

The microcontroller then
* receives an RC6 signal
* translates the RC6 command code to a SIRC command code that a PS2 can understand
* sends out an (unmodulated) SIRC signal to the PS2's microcontroller.

## Configuration
In the `user_config.h` file there are many options you can set - which pins to use for what function, where to store the lookup table, etc. Detailed descriptions are in the file itself.

Edit and run `generate_compressed_lookup.py` to create your own lookup table for translating codes.

See `extras` folder for images of how to connect the ATtiny to your PS2 (at least on the 2 models that I have) and an example of how to send EEPROM programming messages to the ATtiny (if you decide to store your lookup table in EEPROM).

## Implementation details
I wrote the RC6 reception part of this program from scratch - mostly because I haven't found an implementation that works on an Attiny13.

Code space was probably the biggest problem, next to having only one timer. In the end I managed to squeeze everything in and almost hit the limit. There isn't much space if you want to have a big lookup table (about 128 bytes remaining for that).

The program uses the single timer on the ATtiny for both receiving and sending in such a way to make it possible to receive and send simultaneously.

## Why
I'm to lazy to get out of bed and press the power button on my PS2. Also, because OPL supports Bluetooth controllers now, I needed some way to select the game I want to play.without using the wired controller. I have a lot of ATtiny13-s so I decided to take one and make it befriend the PS2 :)

