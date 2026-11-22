#pragma once
#include <stdint.h>
extern int g_servo_us[32];          // pulse width per pin (0 = not driven)
struct Servo {
    int pin = -1;
    bool attached() const { return pin >= 0; }
    uint8_t attach(int p, int = 544, int = 2400) { pin = p; return 1; }
    void detach() { if (pin >= 0) g_servo_us[pin] = 0; pin = -1; }
    void writeMicroseconds(int us) { if (pin >= 0) g_servo_us[pin] = us; }
};
