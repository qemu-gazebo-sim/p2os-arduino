## What is necessary to use it with the robot?

![Hardware setup](./docs/diagramas-test_setup-hardware.drawio.png)


1. Microcontroller (Arduino / ESP32 / etc)
- It uses Serial TX (Transmit) and Serial RX (Receive) pins to communicate.
- Microcontroller from arduino, esp32 usually operates using serial TTL (Transistor-Transistor Logic) level serial signals (typically 3.3V or 5V).

2. TTL to RS232 Converter
- This module converts TTL serial signals (from the microcontroller) to RS232 level signals, which are required by the Pioneer 2 robot.
    - Chip example: max3232
- Directions:
    - TX (from microcontroller) → RS232 RX (to Pioneer)
    - RX (to microcontroller) ← RS232 TX (from Pioneer)

3. DB9 RS232 Connector
- This is a standard RS232 serial port, usually used for rs232 serial communication.
- RS232 signals are typically ±12V (vs. TTL's 0–5V or 0–3.3V).

4. Pioneer Robot (Pioneer 1, Pioneer 2)
- The robot communicates via a DB9 RS232 interface.
- It uses the P2OS protocol (a message-based protocol) to send and receive commands/data.
- Expects the RS232 voltage levels.