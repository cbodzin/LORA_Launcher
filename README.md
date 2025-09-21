# LORA Launcher

An ESP32-based remote launch controller to support M, N, and O motor distances. Comes in two flavors:

## RFM69-based
Designed for the RFM69HCW radio, this uses a controller (`NODEID 1`) that can support up to 4 remote launchers (`NODEID 2-5`).  Range of ~300m, which makes it suitable for M and N motors.

## RF95
Designed for the RF95 LoRa radio, this is solely for a pair (`CONTROLLER` and `REMOTE`).  Range up to 2km with line-of-sight, allowing for the 1,500' distance required for O motors.
