# NavPlotter node for the Raspberry Pi Pico 2 W

A NavPlotter without a screen: a Raspberry Pi Pico 2 W with the Waveshare Pico-LoRa-SX1262 module (868 MHz) and an external GPS module. It does the navigation work and a phone shows it, through a web app served over the board's own WiFi network. It is the same node as the Heltec one of the NavPlotter family, and talks to all the others.

**Not for navigation.**

## What it does

- reads the GPS and sends its position over LoRa, in the packet format shared by all NavPlotters (`include/traffic_packet.h`);
- receives the other nodes, relays their packets (mesh) and computes the collision alarm (CPA / TCPA);
- creates the WiFi network `NavPlotter-<name>` (password `navplotter`) and serves the web app at `http://192.168.4.1`: map, traffic, waypoints, routes, compass, setup;
- a NavPlotter display (for example the Waveshare ESP32-S3-Touch-LCD-4 one) can use it as its node.

## Hardware

| Part | Connection |
|---|---|
| Waveshare Pico-LoRa-SX1262 (868M) | plugs on the Pico: SPI1 SCK GP10, MOSI GP11, MISO GP12, CS GP3, RESET GP15, BUSY GP2, DIO1 GP20. `ANT_SW` jumper on A (3V3), as shipped |
| GPS module (NMEA, any common speed) | GPS TX -> GP1, GPS RX -> GP0 (optional), 3V3 and GND |
| micro SD module, optional (chart tiles in `/tiles/z/x/y.jpg`) | MISO GP16, CS GP17, SCK GP18, MOSI GP19 |

The board must be an original Raspberry Pi Pico 2 W (or Pico W, changing `board` to `rpipicow` in `platformio.ini`). Boards sold as "Pico W" whose LED answers on GP25 have a different WiFi chip and do not work with this firmware: it stops when the WiFi is switched on.

## Differences from the Heltec node

- No display: the on-board LED shows the status (a short flash every 2 s = no GPS fix; on with a short dip = fix; fast blinking = collision alarm) and a status line is written to the USB serial port every 5 s.
- The board cannot keep its own network on while it is joined to another one. In the "boat WiFi" scenario its own network is switched off; if the boat WiFi is not found within 20 s, its own network comes back, and the boat WiFi is looked for again every 3 minutes while nobody is using the node's network.
- The settings live in a small text file on the internal flash. The name can also be set from the computer: type `name <name>` on the serial port.
- The battery voltage is not measured.

## Building

```
~/.platformio/penv/bin/pio run -t upload
```

The first time, the board must be in loader mode: hold BOOTSEL while plugging in the USB cable (with a battery on the LoRa module the board never switches off, so take the module off first). After that, uploads work without the button.

`probe/` holds a small firmware that only switches the WiFi on, to check a board.
