# MicroENV-BLE
This uses a Nano 33 BLE Sense as a portable weather monitor connecting to BLE - nRF Connect on Android works best as it displays DEC values. Uses an SPS30 particulate matter sensor as well.
Materials required:
Nano 33 BLE Sense
SPS30 sensor (Sparkfun sells them with ZHR-5 connectors as well)
Connections:  (SPS30 --> Sense)
TX --> RX0
RX --> TX1
5V --> Vin
GND --> GND
Process:
Open the Arduino IDE
Then, import the code from the .ino file above
Then upload it to the Sense

