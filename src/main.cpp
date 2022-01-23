#include <avr/io.h>                       // for GPIO
#include <avr/interrupt.h>                // for interrupts
#include <avr/pgmspace.h>                 // to store data in program memory
#include <avr/sleep.h>
#include <util/delay.h>

#include "user_config.h"

#define BSET(reg, bit) do { reg |= _BV(bit); } while (0)
#define BCLR(reg, bit) do { reg &= ~_BV(bit); } while (0)
#define PH(pin) BSET(PORTB, pin)
#define PL(pin) BCLR(PORTB, pin)
#define PRESCALER_VALUE 4

constexpr uint16_t prescaler_for_value()
{
  static_assert(PRESCALER_VALUE != 0 && PRESCALER_VALUE < 6);
  return
    PRESCALER_VALUE == 1 ? 1 :
    PRESCALER_VALUE == 2 ? 8 :
    PRESCALER_VALUE == 3 ? 64 :
    PRESCALER_VALUE == 4 ? 256 :
    PRESCALER_VALUE == 5 ? 1024 : 0;
}

constexpr double us_per_tick = 1000000. * prescaler_for_value() / (F_CPU);   // number of microseconds per TCNT0 tick

// Sony SIRC protocol timings
constexpr uint8_t SonySHigh = 2400.0/us_per_tick;
constexpr uint8_t Sony1High = 1200.0/us_per_tick;
constexpr uint8_t Sony0High =  600.0/us_per_tick;
constexpr uint8_t SonyLow =    600.0/us_per_tick;

/**
 * Bit OCIE0B in register TIMSK0 turns on the timer interrupt that is used for timeout.
 * We also use this bit to indicate that signal reception is running.
 */

static const uint8_t RC6_msg_bits = 21;  // Number of bits in an RC6 message (without leader pulse, including start bit)
constexpr double RC6_t_avg = 444.4/us_per_tick;  // Duration of T in timer ticks. All pulses in RC6 have a duration multiple of T.
constexpr double RC6_leader_M= 6 * RC6_t_avg;  // Duration of mark at the start of leader.
constexpr double RC6_leader_S= 2 * RC6_t_avg;  // Duration of space at the end of leader.
constexpr uint8_t margin_high(double x, bool relaxed_timing) { return x + 0.5 + 0.3 * x; }
constexpr uint8_t margin_low (double x, bool relaxed_timing) { return x + 0.5 - 0.3 * x; }

// Use a hardware register for analog comparator (unused in this application) to store status of receiving
// This saves RAM and code bytes (because AVR has instructions specifically for bit twiddling I/O registers)
#define g_status ACSR
enum STATUS_FLAGS {
  STATUS_RX_COMPLETE = _BV(ACIS1),
  STATUS_RX_ERROR = _BV(ACBG),
  STATUS_TOGGLE = _BV(ACIS0),
};

void reset_state(uint8_t reason)
{
  if (reason != 0) {
    g_status |= STATUS_RX_ERROR;
  } else {
    g_status |= STATUS_RX_COMPLETE;
  }

  BCLR(TIMSK0, OCIE0B);
}

/**
 * We abuse PCMSK register to store the bit for activating learn mode.
 * PCINT5 is an interrupt corresponding to the reset pin - this pin is unused here and is hard to use in general
 * (pulling it low always causes a reset unless you burn a fuse, which also prevents programming with a normal ISP).
 */
#define learn_mode_reg PCMSK
#define LEARN_MODE_ON PCINT5

uint8_t g_rc6_2t_meas = 0;
uint16_t g_data_received = 0;
ISR(PCINT0_vect)
{
  static uint16_t data = 0;  // Buffer for remote control data. 16 bits is enough because first 5 bits don't contain much interesting stuff.
  static uint8_t last_counter = 0;  // Used for obtaining time difference since last interrupt
  static bool phase = false;  // If false, ignore state after this edge. If true, capture pin state after this edge.
  static uint8_t edge = 0;  // Edge *and* bit counter. If edge <= 4, counts signal edges. If edge >= 5, edge - 4 equals the number of bits captured.

  #define BIT_EDGE(x) (x + 4)  // Value that `edge` will have after capturing x-th bit
  #define IS_CORRECT_LENGTH(got, expected, relax) ((got) <= margin_high(expected, relax) && (got) >= margin_low(expected, relax))

  uint8_t cntr = TCNT0;  // Capture counter into local variable (avoid multiple read of a volatile memory address)
  OCR0B = cntr + (uint8_t)(7 * RC6_t_avg);  // Set timeout
  if (bit_is_clear(TIMSK0, OCIE0B)) {  // If capture is not running yet (and timeout is disabled)
    edge = 0;  // Reset state
    BSET(TIFR0, OCF0B);  // Clear interrupt flag so it doesn't fire immediately (it's cleared by writing 1)
    BSET(TIMSK0, OCIE0B);  // Start capture (and enable timeout)
  }

#ifdef PASSTHROUGH_ENABLED
  if (bit_is_clear(TIMSK0, OCIE0A)) { // If sending is not running
    // Pass through input data to output
    if (bit_is_set(PINB, IR_OUT)) {
      PH(OUTPUT_PIN);
    } else {
      PL(OUTPUT_PIN);
    }
  }
#endif

  if (edge < 4) { // If edge < 4, count every edge of the signal
    edge++;
  }

  /**
   * At this point:
   * edge == 1 indicates start of leader mark,
   * edge == 2 indicates end of leader mark and start of leader space
   * edge == 3 indicates end of leader space
   * edge == 4 indicates edge in the middle of start bit
   * edge >= 5 indicates we are capturing data bits. E.g. edge == 5 means that start bit has been captured.
   */

  uint8_t diff = cntr - last_counter;
  if (edge == 1) {
    // On edge 1 nothing else needs to be done
  } else if (edge == 2) {
    if (!IS_CORRECT_LENGTH(diff, RC6_leader_M, 1)) {
      reset_state(1);
      return;
    }
  } else if (edge == 3) {
    if (!IS_CORRECT_LENGTH(diff, RC6_leader_S, 0)) {
      reset_state(2);
      return;
    }

    g_rc6_2t_meas = diff; // Used for oscillator calibration
    phase = false;  // Reset capture state
  } else if (edge >= BIT_EDGE(RC6_msg_bits)) {
    // Captured 21 bits in total. We only remember the last 16, but that's okay.
    goto have_result;
  } else {
    // Determine what length of pulse we got
    if (IS_CORRECT_LENGTH(diff, RC6_t_avg, 0)) {
      // Short pulse always inverts phase
      phase = !phase;
    } else if (IS_CORRECT_LENGTH(diff, 2 * RC6_t_avg, 0)) {
      if (edge == BIT_EDGE(4) || edge == BIT_EDGE(5)) {
        // On bits 4 and 5 a long pulse inverts phase
        phase = !phase;
      }
      // Otherwise keeps phase the same
    } else if (IS_CORRECT_LENGTH(diff, 3 * RC6_t_avg, 0)) {
      if (edge != BIT_EDGE(4) && edge != BIT_EDGE(5)) {
        // On bits 4 and 5 a very long pulse keeps phase the same
        // On other bits, it's invalid
        reset_state(4);
        return;
      }
    } else {
      reset_state(5);
      return;
    }

    if (phase) {
      // We are now after the edge in the middle of the bit.
      // If bit is a 1 the transmitter is sending a mark, if bit is 0 sending a space.
      // But receiver outputs low voltage if mark is received - so high state means 1 and low state means 0.
      uint8_t incoming = bit_is_set(PINB, IR_OUT) ? 1 : 0;
      // Push data into the buffer
      data <<= 1;
      data |= incoming;
      edge++;
      if (edge == BIT_EDGE(4) && ((data & 0b111) != 0)) {
        // We've just captured the third "mode" bit
        // We can only interpret mode 0 though
        // Mode 5 indicates that someone wants to send us data to put in EEPROM
        if (LOOKUP_OPT == LOOKUP_IN_EEPROM && (data & 0b111) == 5) {
          BSET(learn_mode_reg, LEARN_MODE_ON);
        } else {
          reset_state(6);
          return;
        }
      } else if (edge == BIT_EDGE(5)) {
        // We got the toggle bit in case you need it
        g_status = incoming;
      } else if (edge >= BIT_EDGE(RC6_msg_bits)) {
        // We read the last bit
        if (incoming) {
          // If it's 1, this is the last edge. Message received, disable the timeout.
          goto have_result;
        }
        // If it's 0, there's another edge coming. Technically speaking we have the message, but we still need to handle that last edge.
      }
    }
  }

  last_counter = cntr;
  return;
have_result:
  g_data_received = data;
  reset_state(0);
  return;
}

ISR(TIM0_COMPB_vect)
{
  reset_state(7);
}

// We abuse the EEPROM data register for an unrelated purpose to save code bytes.
// At the point where send_sirc_code() is called no one else can mess with this register.
#define g_send_bit EEDR // 7 lower bits: number of bits transmitted, top bit: last pulse transmitted was a mark
uint8_t send_data[3];

void send_sirc_code(uint8_t cmd)
{
  // Reset prescaler. By doing this we ensure that we are at the beginning of the timer tick
  // when we capture its state. It might mess a bit with message reception, but not much.
  // At this point we assume that we finished receiving already.
  BSET(GTCCR, PSR10);
  OCR0A = TCNT0 + SonySHigh;
  PL(OUTPUT_PIN);
  BSET(TIFR0 , OCF0A);  // Clear interrupt flag so it doesn't fire immediately (it's cleared by writing 1)
  g_send_bit = 0x80;
  send_data[0] = cmd & 0x7F;
  send_data[1] = (cmd & 0x80) ? 0xAD : 0x9D;
  send_data[2] = (cmd & 0x80) ? 0x0D : 0x04;
  BSET(TIMSK0, OCIE0A);
  loop_until_bit_is_clear(TIMSK0, OCIE0A);  // Wait for sending to finish
}

// IR send bitbang routine
ISR(TIM0_COMPA_vect)
{
  // This should be called right after a timer tick, so we have plenty of time before the next tick
  uint8_t current = TCNT0;
  if (bit_is_set(g_send_bit, 7)) {
    // Switch output off
    PH(OUTPUT_PIN);
    BCLR(g_send_bit, 7);
#ifdef PASSTHROUGH_ENABLED
    if (g_send_bit >= 20) {
      // Leave a few ms of space at the end.
      // We do it by letting the counter roll over and hit the same value again.
      // Make sure that the length of the counter is between 5 and 10 ms.
      static_assert(5000 / us_per_tick < 256);
      static_assert(10000 / us_per_tick >= 256);
    } else {
      OCR0A = current + SonyLow;
    }
#else
    OCR0A = current + SonyLow;
#endif
  } else {
    uint8_t send_bit = g_send_bit;  // "Cache" the result to save 2 instructions
    if (send_bit >= 20) {
      // Finished sending
      BCLR(TIMSK0, OCIE0A);
      return;
    }

    // Bang out next bit
    PL(OUTPUT_PIN);
    uint8_t send_i = send_bit / 8;
    if (send_data[send_i] & 1) {
      OCR0A = current + Sony1High;
    } else {
      OCR0A = current + Sony0High;
    }

    send_data[send_i] >>= 1;
    g_send_bit += (_BV(7) + 1); // set top bit and add 1
  }
}

#if LOOKUP_OPT == LOOKUP_IN_PROGMEM
const uint8_t PROGMEM lookup_compr[] = {
  99,  218, 26,  219, 216, 44,  217, 209,
  208, 210, 221, 252, 49,  225, 48,  50,
  226, 57,  56,  248, 100, 235, 212, 214,
  215, 213, 222, 239, 223, 220, 211, 255,
  251, 174,
};
#endif

uint8_t EEPROM_read(uint8_t addr)
{
  // Skip waiting for readiness; there is no way we can start a read while an operation is in progress
  EEARL = addr;
  BSET(EECR, EERE);
  return EEDR;
}

void EEPROM_write(uint8_t addr, uint8_t data)
{
  EECR = 0;
  EEARL = addr;
  EEDR = data;
  BSET(EECR, EEMPE);
  BSET(EECR, EEPE);
  loop_until_bit_is_clear(EECR, EEPE);
}

uint8_t code_lookup(uint8_t target_code, uint8_t last_status)
{
#ifdef SW_POWER_TOGGLE
  static bool power_toggle = false;
#endif

#if LOOKUP_OPT == NO_LOOKUP
  uint8_t mapped = target_code;
#elif (LOOKUP_OPT == LOOKUP_IN_EEPROM) || (LOOKUP_OPT == LOOKUP_IN_PROGMEM)
  uint8_t mapped = 0xFF;
  uint8_t current_output_code = 0;
#if LOOKUP_OPT == LOOKUP_IN_EEPROM
  uint8_t size = EEPROM_read(0);
  if (size == 0 || size > 63) {
    size = 0;
    //mapped = target_code;
  }
#else
  uint8_t size = sizeof(lookup_compr) / sizeof(lookup_compr[0]);
#endif
  for (uint8_t i = 0; i < size; i++) {
#if LOOKUP_OPT == LOOKUP_IN_EEPROM
  uint8_t v = EEPROM_read(i+1);
#else
    uint8_t v = pgm_read_byte(&lookup_compr[i]);
#endif
    if (v >= 0xE0) {
      current_output_code += v - 0xE0;
    } else if (target_code == current_output_code) {
      mapped = v;
      break;
    } else if (target_code < current_output_code) {
      break;
    }

    current_output_code++;
  }
#endif

  if (mapped == (0x2E | 0x80)) { // PS2 Power on code
#ifdef SW_POWER_TOGGLE
    mapped |= power_toggle ? 1 : 0;
    power_toggle = !power_toggle;
#endif

#ifdef HW_POWER_TOGGLE
    mapped |= bit_is_set(PINB, RED_LED_POS) ? 0 : 1;
#endif

#ifdef RC6_POWER_TOGGLE
    static_assert(STATUS_TOGGLE == 1, "The code below only works if status is 1");
    mapped |= (last_status & 1);
#endif
  }

  return mapped;
}

#ifdef AUTOCALIBRATION_ENABLED
uint8_t osccal_default;
/**
 * @param meas Measured time of 2t
 * @note This takes about 60 bytes
 */
void adjust_oscillator(uint8_t meas)
{
  static int8_t osc_adj = 0;
  const uint8_t target_2t_length = (2 * RC6_t_avg);
  const int8_t adjustment_upper_bound = 12; // Upper bound is lower because the OSCCAL value => frequency is non-linear
  const int8_t adjustment_lower_bound = -24;
  if (meas < target_2t_length - 1 && osc_adj < adjustment_upper_bound) {
    osc_adj++;
    OSCCAL = osccal_default + osc_adj;
  } else if ((meas > target_2t_length + 1 && osc_adj > adjustment_lower_bound)) {
    osc_adj--;
    OSCCAL = osccal_default + osc_adj;
  }
}
#else
void adjust_oscillator(uint8_t meas) { }
#endif

int main(void)
{
#ifdef AUTOCALIBRATION_ENABLED
  osccal_default = OSCCAL;
#endif
  // Set up pins
  DDRB    = _BV(OUTPUT_ACTIVE) | _BV(OUTPUT_PIN);
  PORTB   = _BV(OUTPUT_ACTIVE) | _BV(OUTPUT_PIN) | _BV(IR_OUT);
  // Disable ADC (it's not used) to save power
  BSET(PRR, PRADC);
  // Set up timer
  TIMSK0  = 0;
  TCCR0A  = 0;
  TCCR0B  = PRESCALER_VALUE << CS00;
  // Set up input interrupt
  BSET(PCMSK, IR_OUT);
  BSET(GIMSK, PCIE);
  // Set sleep mode to power down
  MCUCR   = SLEEP_MODE_PWR_DOWN; // Other stuff in the register should be 0
  sei();
  sleep_mode();
  bool stop_sending = false;
  while (1) {
    uint8_t status = g_status;
    if (status & (STATUS_RX_COMPLETE | STATUS_RX_ERROR)) {
      if (!(status & STATUS_RX_ERROR)) {
        adjust_oscillator(g_rc6_2t_meas);
        uint8_t addr = g_data_received >> 8;
        uint8_t cmd = g_data_received & 0xFF;
#if LOOKUP_OPT == LOOKUP_IN_EEPROM
        bool learn_mode = bit_is_set(learn_mode_reg, LEARN_MODE_ON);
        BCLR(learn_mode_reg, LEARN_MODE_ON);
#else
        bool learn_mode = false;
#endif
        if (learn_mode) {
          PL(OUTPUT_PIN);
          EEPROM_write(addr, cmd);
          PH(OUTPUT_PIN);
        } else if (addr == 0) {
          if (cmd == STOP_SENDING_BTN) {
            stop_sending = !stop_sending;
          }

          if (!stop_sending) {
            uint8_t ps2_code = code_lookup(cmd, status);
            if (ps2_code != 0xFF) {
              PH(OUTPUT_ACTIVE);
              _delay_ms(1);
              for (int i = 0; i < 3; i++) { // PS2 expects the same message sent 3 times
                send_sirc_code(ps2_code);
                _delay_ms(SIRC_REPEAT_DELAY);
              }

              PL(OUTPUT_ACTIVE);
            }
          }
        }
      }

      g_status = 0;
      if (bit_is_clear(TIMSK0, OCIE0B)) {
        sleep_mode();
      }
    }
  }
}
