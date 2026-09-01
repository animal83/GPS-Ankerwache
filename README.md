# GPS-Ankerwache im klein Vormat
Die Idee dahinter ist das man eine kleine mobilde Ankerwache hat, die autag läuft an kein Funkgerät, Bus-System etc. angeschlossen werden muss und länger lauft als ein Handy-Akku.

## Material 
- 1 x Arduino Mega 2560 Rev3
- 1 x Adafruit Channels blau – GPS-Empfänger-Modul
- 1 x AZDelivery OLED I2C Display 128 x 64 Pixel  
- 1 x AZDelivery OLED I2C Display 128 x 32 Pixel  
- 1 x AZDelivery Real Time Clock RTC DS3231 I2C
- 1 x WayinTop Retary Encoder Modul KY-040 360 Grad

## Benötigte Bibliotheken

Über den Bibliotheksverwalter der Arduino IDE installieren:

- **U8g2** von oliver, enthält `U8x8lib.h` für das OLED-Display

Bereits im Arduino-AVR-Core enthalten und nicht separat zu installieren:

- `Arduino.h`
- `SPI.h`
- `HardwareSerial` für `Serial3`

## GPS-Verkabelung

Der Sketch verwendet die Hardware-Schnittstelle `Serial3` des Arduino Mega:

- GPS-TX an Mega-Pin 15 (`RX3`)
- GPS-RX an Mega-Pin 14 (`TX3`)
