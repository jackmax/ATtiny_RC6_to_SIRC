#pragma once

/* Pins used in the program. There are no limits which pin does what function. */
// Signal from IR receiver.
#define IR_OUT 3
// Signal to PS2 microcontroller, Mechacon, Dragon, whatever you call it.
#define OUTPUT_PIN 0
// Goes high when translator starts transmitting, low after transmission is finished.
// Not necessary if you don't plan to have a hardware passthrough.
#define OUTPUT_ACTIVE 1
// Connected to red LED control signal (active high, therefore pin is low when PS2 is on).
#define RED_LED_POS 4

/**
 * SIRC commands for the PS2 are 7 bits long, but they use one of 2 addresses.
 * RC6 commands are 8 bits long, so this program treats their top bit as address selector for SIRC.
 * When top bit is 0, commands are sent with address 0x093A, if 1 with 0x1B5A.
 *
 * The program has an option to translate an RC6 command to any SIRC command you choose.
 * The lookup table can be generated with generate_compressed_lookup.py
 *
 * NO_LOOKUP - Don't do code translation (send code with the same binary value)
 * LOOKUP_IN_EEPROM - Lookup table is in EEPROM. This limits it to 63 bytes, but allows reprogramming.
 * LOOKUP_IN_PROGMEM - Lookup table is in Flash. It can be any size as long as it fits alongside the program.
 */
#define NO_LOOKUP 0
#define LOOKUP_IN_EEPROM 1
#define LOOKUP_IN_PROGMEM 2

#define LOOKUP_OPT LOOKUP_IN_EEPROM

/**
 * PS2 has two different power codes - one for power on, another for power off.
 * Typically remotes only have one power button, so the translator can toggle between
 * sending power on and power off codes to make it dual function.
 * NB: PS2 doesn't react to power on when it's already on, or to power off if it's already off.
 *
 * There are 4 options:
 * 1. Don't do anything - Use if you map two buttons, one to power on, one to power off
 * 2. SW_POWER_TOGGLE - Every time the button mapped to "Power on" is pressed,
 * the software toggles between sending a power on and power off command.
 * Cost: 20 bytes
 * 3. HW_POWER_TOGGLE - A pin (connected e.g. to the green LED) is read just before sending.
 * If pin is high (LED off) power on command is sent, otherwise power off command is sent.
 * You'll have to solder an extra pin.
 * Cost: 16 bytes
 * 4. RC6_POWER_TOGGLE - Uses the "toggle bit" sent by the remote control as part of RC6 protocol.
 * It switches its value every time the button is pressed (but keeps its value while the button
 * is pressed down). Note that not all remotes send the toggle bit as intended (esp. copying remotes).
 * Cost: 10 bytes
 */
//#define SW_POWER_TOGGLE
#define HW_POWER_TOGGLE
//#define RC6_POWER_TOGGLE

// If this button is pressed, the translator will stop sending out translated inputs
// and start sending them when it's pressed again
#define STOP_SENDING_BTN 0x3c

// If enabled, all pulses from the input will be forwarded to the output (unless a translated code is being transmitted)
// This software passthrough doesn't work very well (seems to miss some pin changes).
//#define PASSTHROUGH_ENABLED

// If enabled, adjust internal RC oscillator to sync with the oscillator in the remote.
#define AUTOCALIBRATION_ENABLED

// Messages should be sent in a 45 ms raster. We can't do it exactly, because the delay at the end
// should depend on the contents of the message. This is a conservative estimate based on average message length.
#define SIRC_REPEAT_DELAY 25
