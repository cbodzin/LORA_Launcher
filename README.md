# LORA Launcher

An ESP32-based remote launch controller to support M, N, and O motor distances. Designed for the RF95 LoRa radio, this is solely for a pair (`CONTROLLER` and `REMOTE`).  Range up to 2km with line-of-sight, allowing for the 1,500' distance required for O motors.

![Controller and Remote](LORA_Launcher.JPG "Controller and Remote")

## Parts used

1. Two ESP32s, one for the controller and one for the remote.  You'll a lot of usable pins, so I used [this one](https://www.amazon.com/dp/B08D5ZD528)
2. Two RFM95W Breakouts.  I used [these from Adafruit](https://www.adafruit.com/product/3072).  If you don't need the longest range then you can go with the [simple spring antenna](https://www.adafruit.com/product/4269) but I'd recommend this [uFL SMT connector](https://www.amazon.com/dp/B0CB6D28R5) with [this SMA antenna](https://www.amazon.com/dp/B0FM4KS6KY)
3. This (or any ST7789-based) [TFT display](https://www.adafruit.com/product/4383)
4. Momentary pushbutton switches for the status, launch, and continuity test buttons.
5. Toggle switches for the power and arming switches.
6. An [IRLZ44N MOSFET](https://www.amazon.com/dp/B0CBKH4XGL)
7. A [5V Relay Breakout](https://www.amazon.com/dp/B00LW15A4W)
8. If you want to run the remote off of a single battery then you should get [5V Buck Converters](https://www.amazon.com/dp/B0F1WB3LJ5).
9. A decent battery for ignition, either 7.4V or 12V recommended.
10. Two [RGB LEDs](https://www.amazon.com/dp/B01C19ENDM) and [holders](https://www.amazon.com/dp/B083QB966V) for them.
11. A bunch of 10K, 1K, and 220 resistors.
12. A [passive buzzer](https://www.amazon.com/Gikfun-Terminals-Passive-Electronic-Arduino/dp/B01GJLE5BS) for the remote.
13. A [micro-USB passthrough port](https://www.amazon.com/dp/B0BMDSHJ88) makes it easy to update the firmware and power via a USB powerbank.
14. Project box for everything to go in.  I used [this one](https://www.amazon.com/dp/B08KWD8TFY)