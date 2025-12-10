| Supported Targets | ESP32 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# IR NEC Transceiver

(See the README.md file in the upper level 'examples' directory for more information about examples.)

[NEC](https://www.sbprojects.net/knowledge/ir/nec.php) is a common use IR protocol, this example creates a TX channel and sends out the IR NEC signals periodically. The signal is modulated with a 38KHz carrier. The example also creates an RX channel, to receive and parse the IR NEC signals into scan codes.

It is also have a built in web server for triggering transmission of NEC scan codes.

## Hardware Required

- A development board with supported SoC mentioned in the above `Supported Targets` table
- An USB cable for power supply and programming
- A 5mm infrared LED (e.g. IR333C) used to transmit encoded IR signals
- An infrared receiver module (e.g. IRM-3638T), which integrates a demodulator and AGC circuit

## Build and Flash

To configure, run `idf.py menuconfig`. There you can configure the project (e.g. wifi password for connecting to router, etc.).

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type `Ctrl-]`.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.
