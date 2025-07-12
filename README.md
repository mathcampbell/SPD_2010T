# SPD2010-Touch-Arduino

> **A pure-Arduino driver for the Solomon Systech SPD2010 capacitive
> touch-panel IC used on the waveshare 1.46@ esp32 dev-board...tested on ESP32-S3
> but portable to any MCU with
> `Wire`.**  
> No ESP-IDF glue, no heavyweight framework, no drama – drop the two
> files into your project and go.

---

## Why this repo exists 

The Waveshare "ESP32-S3 1.46inch Round Display Development Board" ships with an
ESP-IDF/Arduino wrapper that drags in a whole mass of extra components files that 
are all limmited/demo drivers and still refuses to compile on stock PlatformIO.  
I spent **three days** reverse-engineering the protocol (and reading
MicroPython sources) so you don’t have to.

https://www.waveshare.com/product/arduino/boards-kits/esp32-s3/esp32-s3-touch-lcd-1.46b.htm
---

## Features

| ✔ | Capability |
|---|------------|
| Single-file **`SPD2010Touch.cpp / .h`** drop-in |
| Multi-touch (≤5 points × weight) |
| Swipe / scroll gesture byte |
| IRQ-driven, non-blocking polling (no `delay()` loops) |
| Optional reset via **Adafruit XCA9554** I/O expander |
| Works at 400 kHz I²C on ESP32-S3, should down-clock happily to 100 kHz |
| Compile-time `SPD_DEBUG` switch – zero overhead when off |

---

## Wiring

| SPD2010 pin | MCU (ESP32-S3) | Notes |
|-------------|---------------|-------|
| **SDA**     | GPIO 11        | Pull-ups already on Waveshare PCB |
| **SCL**     | GPIO 10        | 400 kHz OK |
| **INT**     | GPIO 4         | Active-low, attachInterrupt FALLING |
| **RST**     | *optional* via XCA9554 pin 0 | Tie to expander if free GPIOs are scarce |

*I²C address is fixed at **`0x53`**.*

---

## Quick start

```cpp
#include <Wire.h>
#include <Adafruit_XCA9554.h>
#include "SPD2010Touch.h"

constexpr uint8_t INT_PIN   = 4;
constexpr uint8_t RST_EXPin = 0;          // pin-0 on the XCA9554
constexpr uint8_t XCA_ADDR  = 0x20;       // A0..A2 = GND

Adafruit_XCA9554 expander;
SPD2010Touch     touch(Wire, RST_EXPin, INT_PIN, &expander);

void setup()
{
  Serial.begin(115200);
  Wire.begin(11 /*SDA*/, 10 /*SCL*/, 400000);

  expander.begin(XCA_ADDR);               // reset line lives here
  touch.begin();                          // prints FW version

  Serial.println("touch driver ready");
}

void loop()
{
  static uint32_t last;
  if (millis() - last < 10) return;       // 100 Hz poll
  last = millis();

  uint16_t x, y; uint8_t w;
  if (touch.getTouch(x, y, w))
    Serial.printf("x=%3u  y=%3u  weight=%u\n", x, y, w);
}
```

## API

Method	Description
bool begin()	initialise I²C, reset line, read FW
bool available()	true if an IRQ is pending
bool read(TouchData &d)	fill full struct (multi-touch)
bool getTouch(uint16_t &x, uint16_t &y, uint8_t &w)	single-finger helper
uint8_t getTouchPoints(TouchPoint *buf, uint8_t max)	array helper
bool isTouched()	any finger on glass
uint8_t getGesture()	swipe / scroll code
void setInterruptCallback(cb)	attach your own ISR if desired

See SPD2010Touch.h for the TouchData / TouchPoint layout.

## Building
PlatformIO
ini
Copy
Edit
[env:esp32-s3]
platform      = espressif32
board         = esp32-s3-devkitm-1
framework     = arduino
monitor_speed = 115200
build_flags   = -DSPD_DEBUG=0      ; set =1 for verbose prints
Just copy SPD2010Touch.cpp/.h into /src.

Arduino IDE 2.x
Sketch ➜ Add File… ✓

Tools ➜ Core = ESP32 (3.0+ recommended)

## Credits
Waveshare example (ESP-IDF) for the original register set

Kevin Schlosser for the
MicroPython driver that confirmed the controller state-machine
https://github.com/lvgl-micropython/lvgl_micropython/blob/main/api_drivers/common_api_drivers/indev/spd2010t.py

ChatGPT o3 for rubber-ducking through way too many crahses and failed calls.


