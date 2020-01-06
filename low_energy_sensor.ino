/*
 *  Low Energy Sensor v1.0
 *  mcu:          ATtiny85
 *  sensor:       BME280 (Bosch Sensortec) [ONLY in i2c, NOT in spi]
 *  transmission: RF 433Mhz (Olivier Lebrun's Oregon Scientific Emitter)
 *
 *
 * Copyright (C) 2018 conurb@online.fr
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *  
 *
 *  Wiring (ATtiny85):
 *                            +-------+
 *                           1|*      |8  VCC
 *                           2|       |7  SCL (BME280)
 *  Data TX RF 433Mhz - PB4  3|       |6
 *                      GND  4|       |5  SDA (BME280)
 *                            +-------+
 *
 *     
*/

#include "configuration.h"
#include "oregon_scientific_emitter.h"
#include "bme280_tiny_i2c.h"

BME280 bme;

inline void mcu_set_watchdog()
{
  MCUSR &= ~(1 << WDRF);               // clear the reset flag (DS 8.5.1)
  WDTCR |= (1 << WDCE) | (1 << WDE);   // Watchdog Timer Prescale (DS 8.4.1.2 & 8.5.2)
  WDTCR = (1 << WDP3) | (1 << WDP0);   // set to roughly 8s (max) (DS 8.5.2, Table 8-3)
  WDTCR |= (1 << WDIE);                // enable interrupt mode
}

inline void mcu_set_power_down_mode()
{
  MCUCR |= (1 << SM1) | (1 << SE);     // select Power-down Mode (DS 7.5.1)
}

inline void mcu_sleep(uint8_t wdt_period)
{
  // Reduce Power Consumption (Low Energy):
  // - ADC off
  // DS 7.4.1 & 17.13.2
  ADCSRA &= ~(1 << ADEN);

  for (uint8_t i = wdt_period; i > 0; i--)
    __asm__ __volatile__("sleep");
  
  // Enable ADC (Analog Digital Converter)
  // needed for measurements with bandgap voltage reference : 
  // 25 ADC clock cycles vs 13 if ADC is not re-enabled
  ADCSRA |= (1 << ADEN);
}

inline int mcu_read_vcc()
{  
  // Vbg (select bandgap reference voltage)  
  ADMUX = (1 << MUX3) | (1 << MUX2);
  // DS Table 17-4 : cf (2) for choice '1100'
  // After switching to internal voltage reference the ADC requires 
  // a settling time of 1ms before measurements are stable (take 2ms)
  delay(2);
  ADCSRA |= (1 << ADSC);         // start conversion (DS 17.3.2)
  while (ADCSRA & (1 << ADSC));  // wait while conversion is in progress
  // ADC data
  // DS 17.3 : ADCL has to be read first
  byte low, high;
  low = ADCL;
  high = ADCH;
  return ((uint32_t)1024 * 1100) / ((high << 8) | low);
}

void setup()
{  
  // mcu pin
  // RF TX pin set as output low
  DDRB = (1 << DDB4);
  
  // sensor: bme280
  bme280_init(&bme);

  // oregon scientific protocol
  oregon_init();
  
  // mcu: set watchdog & power-down mode
  mcu_set_watchdog();
  mcu_set_power_down_mode();
}

void loop()
{
  // read values from sensor
  bme280_read_sensor(&bme);

  // prepare message
  oregon_set_battery_level(mcu_read_vcc() > LOW_BATTERY_ALERT);
  oregon_set_temperature(bme.temperature / 100.0);
#if MODE
  oregon_set_humidity(lrintf(bme.humidity / 1024.0));
#endif
#if MODE == 2
  oregon_set_pressure(bme.pressure / 100.0);
#endif

  // send message
  // this is the responsability of the oregon scientific emitter to take care of
  // sending or not all messages (in respect of SEND_ONLY_NEW_MESSAGE option)
  oregon_send_message();

  // go to sleep
  // wdt prescaler has been fixed to roughly 8s by period
  // div by 8 the sleeping time requested by user
  mcu_sleep(SLEEPING_TIME_S / 8);
}

ISR(WDT_vect)
{}
