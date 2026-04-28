#include "samc21.h"
#include "init.hpp"
#include "pins.hpp"
#include "serial.hpp"
#include "timebase.hpp"
#include "waveform.hpp"

int main(void)
{
    sys_init();
    Waveform::init();
    Timebase::init();
    Serial.init(1000000);

    Serial.print("OSC48M  @ 48 MHz  : OK (GCLK0)\r\n");
    Serial.print("XOSC    @ 24 MHz  : OK (GCLK1)\r\n");
    Serial.print("TCC0    @ 375 kHz : OK (PA08 87.5% / PA10 12.5% / PA11 50%)\r\n");
    Serial.print("UART    @ 1 Mbaud : OK\r\n");

    uint32_t last_ms = 0;
    uint32_t count = 0;

    for (;;)
    {
        uint32_t now = Timebase::millis();
        if (now - last_ms >= 100)
        {
            last_ms = now;
            Serial.print("count=");
            Serial.print(count++);
            Serial.print("  uptime=");
            Serial.print(now);
            Serial.print(" ms\r\n");
        }
    }
}
