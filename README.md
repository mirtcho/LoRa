Arduino LoRa
An Arduino library for sending and receiving data using LoRa radios.

Git braches and notes
master branch contain basic functionality that is tested by different users.
develop brach - fully tested by me version. In case of problem please send me an issue report.
Develop brach contains many features and bugs solvings. e.g. Features - support Raspberry PI and arduino , solved bug - low data rate LoRa

Compatible Hardware
ebyte E19-868MS1W
Semtech SX1276/77/78/79 wiring
Pin Name	E19-868MS1W pin#	Arduino DIO#
VCC5V	11	ICSP 5V
GND	1,9,10,18	ICSP GND
SCK	12	ICSP SCK
MISO	13	ICSP MISO
MOSI	14	ICSP MOSI
NSS	15	7
TX_EN	16	2
RX_EN	17	3
NRESET	8	6
DIO0	7	5
DIO1	6	4
note DIO1 not used by software

NSS, NRESET, and DIO0 pins can be changed by using LoRa.setPins(ss, reset, dio0). DIO0 pin is optional, it is only needed for receive callback mode. If DIO0 pin is used, it must be interrupt capable via attachInterrupt(...).

NOTE: Some boards (like the Arduino Nano), cannot supply enough current for the SX127x in TX mode. This will cause lockups when sending, be sure to use an external 3.3V supply that can provide at least 120mA's when using these boards.

Installation
Using the Arduino IDE Library Manager
Choose Sketch -> Include Library -> Manage Libraries...
Type LoRa into the search box.
Click the row to select the library.
Click the Install button to install the library.
Using Git
cd ~/Documents/Arduino/libraries/
git clone https://github.com/mirtcho/LoRa
API
See API.md.

Examples
See examples folder.

License
This libary is licensed under the MIT Licence.