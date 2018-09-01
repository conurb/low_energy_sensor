# low-energy-sensor
ATtiny85 with a BME280 sensor in I2C (mimics an Oregon Scientific sensor)

## Arduino IDE

### Install [SpenceKonde's ATTinyCore](https://github.com/SpenceKonde/ATTinyCore)

Boards Manager URL: http://drazzy.com/package_drazzy.com_index.json (see [Installation](https://github.com/SpenceKonde/ATTinyCore/blob/master/Installation.md) on SpenceKonde's github pages).

**low_energy_sensor WILL NOT WORK with [damellis's attiny](https://github.com/damellis/attiny) library.**

### Compilation's choices

* Type de carte: "ATtiny25/45/85"
* Save EEPROM: "EEPROM Retained"
* Timer 1 Clock: "CPU"
* LTO (1.6.11+ only): "Disabled"
* B.O.D. Level: "B.O.D. Disabled"
* Chip: "ATtiny85"
* Clock: "8 Mhz (internal)"

## Usage

Inside the low_energy_sensor project, open and modify "configuration.h" to obtain what is more suited for you.

(wiring is at the top of "configuration.h")

