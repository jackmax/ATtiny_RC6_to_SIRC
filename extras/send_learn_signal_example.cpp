#include <Arduino.h>
// Requires library z3t0/IRremote
#include <IRremote.hpp>

void setup() {
    Serial.begin(115200);
    pinMode(0, INPUT_PULLUP);
    IrSender.begin(IR_SEND_PIN, true, LED_BUILTIN);
    pinMode(LED_BUILTIN, OUTPUT);
    IrReceiver.start();
}

uint64_t make_data(uint8_t addr, uint8_t data) {
  return
    (5 << 17) |
    (addr << 8) |
    (data << 0);
}

void loop() {
  if (digitalRead(0) == LOW) {
    IrSender.sendRC6(make_data(1, 123), 20);
    delay(10);
    IrSender.sendRC6(make_data(2, 124), 20);
    delay(10);
    IrSender.sendRC6(make_data(0, 2), 20);
    delay(1000);
    }
}
