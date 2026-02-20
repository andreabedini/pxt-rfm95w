# RFM95W LoRa Radio driver

This library lets a micro:bit send and receive text or raw bytes over
long-range LoRa radio using an
[Adafruit RFM95W](https://www.adafruit.com/product/3072) (Semtech SX1276)
breakout board.  No internet connection is required — the modules talk directly
to each other over the air.

## Wiring

Connect the RFM95W breakout to the micro:bit edge connector as follows:

| RFM95W pin | micro:bit pin |
|------------|---------------|
| SCK        | P13           |
| MOSI       | P14 ¹         |
| MISO       | P15           |
| CS / NSS   | P16           |
| RST        | P8            |
| G0 / DIO0  | P1            |
| VIN        | 3V            |
| GND        | GND           |

¹ Some breakouts label this pin **MOSI**; others label it **SDI** or **DI**.

## Basic usage

You need at least two micro:bits with RFM95W modules — one sender, one receiver.
Both must call ``||Lora:LoRa init||`` with the **same band** before communicating.

### Receiver

```blocks
Lora.onReceivedString(function (receivedString) {
    basic.showString(receivedString)
})
Lora.init(Lora.Band.EU868)
```

### Sender

```blocks
Lora.init(Lora.Band.EU868)
basic.forever(function () {
    Lora.sendString("Hello!")
    basic.pause(2000)
})
```

## Checking signal quality

Use ``||Lora:LoRa signal strength||`` and ``||Lora:LoRa received SNR||`` inside
the ``||Lora:on LoRa received||`` handler to judge link quality.  RSSI values
between −70 and −110 dBm are typical; SNR above 0 dB indicates a clean signal.

```blocks
Lora.onReceivedString(function (receivedString) {
    basic.showString(receivedString)
    basic.showNumber(Lora.receivedSignalStrength())
})
Lora.init(Lora.Band.EU868)
```

## Advanced configuration

The default modem settings (SF7, BW 125 kHz, CR 4/5) give a good balance of
speed and range for classroom use.  You can change them after ``init`` for
longer range or more noise tolerance.  **Both ends must use the same settings.**

```blocks
Lora.init(Lora.Band.EU868)
Lora.setSpreadingFactor(Lora.SpreadingFactor.SF10)
Lora.setBandwidth(Lora.Bandwidth.BW125k)
Lora.setCodingRate(Lora.CodingRate.CR48)
Lora.setSyncWord(0x42)
```

Use ``||Lora:LoRa set TX power||`` to increase range at the cost of battery
life, or to reduce it during benchtop testing:

```blocks
Lora.init(Lora.Band.EU868)
Lora.setTxPower(20)
```

## Supported targets

* for PXT/microbit

## License

MIT
