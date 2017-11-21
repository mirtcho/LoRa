# Arduino LoRa


An [Arduino](http://arduino.cc/) library for sending and receiving data using [LoRa](https://www.lora-alliance.org/) radios.

## Compatible Hardware

 * ebyte E19-868MS1W

### Semtech SX1276/77/78/79 wiring

|  Pin   | E19-868MS1W |   Arduino |
|  Name  | Pin #       |    DIO#   |
|--------|:-----------:|----------:|
| VCC5V  |    11       | ICSP 5V   |
| GND    | 1,9,10,18   | ICSP GND  |
| SCK    |    12       | ICSP SCK  |
| MISO   |    13       | ICSP MISO |
| MOSI   |    14       | ICSP MOSI |
| NSS    |    15       |   7       |
| TX_EN  |    16       |   2       |
| RX_EN  |    17       |   3       |
| NRESET |     8       |   6       |
| DIO0   |     7       |   5       |
| DIO1   |     6       |   4       | not used by software

`NSS`, `NRESET`, and `DIO0` pins can be changed by using `LoRa.setPins(ss, reset, dio0)`. `DIO0` pin is optional, it is only needed for receive callback mode. If `DIO0` pin is used, it **must** be interrupt capable via [`attachInterrupt(...)`](https://www.arduino.cc/en/Reference/AttachInterrupt).

**NOTE**: Some boards (like the Arduino Nano), cannot supply enough current for the SX127x in TX mode. This will cause lockups when sending, be sure to use an external 3.3V supply that can provide at least 120mA's when using these boards.

## Installation

### Using the Arduino IDE Library Manager

1. Choose `Sketch` -> `Include Library` -> `Manage Libraries...`
2. Type `LoRa` into the search box.
3. Click the row to select the library.
4. Click the `Install` button to install the library.

### Using Git

```sh
cd ~/Documents/Arduino/libraries/
git clone https://github.com/sandeepmistry/arduino-LoRa LoRa
```

## API

See [API.md](API.md).

## Examples

See [examples](examples) folder.

## License

This libary is [licensed](LICENSE) under the [MIT Licence](http://en.wikipedia.org/wiki/MIT_License).
