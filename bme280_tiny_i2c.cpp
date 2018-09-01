/*
 *  Tiny Library for BME280 sensor (Bosch Sensortec) in I2C
 *  
 *  Copyright (C) 2018 conurb@online.fr
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *  
 *  DS used:
 *  https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280_DS001-12.pdf
 *  
*/

#include <Arduino.h>
#include <Wire.h>
#include "configuration.h"
#include "bme280_tiny_i2c.h"

static void
bme280_write_register(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission((uint8_t)BME280_I2C_ADDR);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

static uint8_t
bme280_read_register(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
  uint8_t  received_bytes  = 0;
  uint8_t  remaining_bytes = len;
  uint8_t* data_received   = data;

  Wire.beginTransmission((uint8_t)BME280_I2C_ADDR);
  Wire.write(reg_addr);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BME280_I2C_ADDR, len);

  while (Wire.available() && remaining_bytes--) {
    *data_received++ = Wire.read();
    ++received_bytes;
  }
  return (received_bytes == len) ? 1 : 0;
}

static uint8_t
bme280_get_calibration_data(BME280* bme)
{
  uint8_t calibration_data[BME280_T_P_CALIB_DATA_LEN] = { 0 };

  // get values from register 0x88 to 0xA1
  uint8_t ret = bme280_read_register(BME280_REG_T_P_CALIB_DATA, BME280_T_P_CALIB_DATA_LEN, calibration_data);
  if (!ret)
    return 0;

  struct calibration_data* cdata = &bme->calib_data;

  cdata->dig_T1 = (uint16_t)BME280_MAKE_WORD(calibration_data[1], calibration_data[0]);
  cdata->dig_T2 = (int16_t)BME280_MAKE_WORD(calibration_data[3], calibration_data[2]);
  cdata->dig_T3 = (int16_t)BME280_MAKE_WORD(calibration_data[5], calibration_data[4]);
  cdata->dig_P1 = (uint16_t)BME280_MAKE_WORD(calibration_data[7], calibration_data[6]);
  cdata->dig_P2 = (int16_t)BME280_MAKE_WORD(calibration_data[9], calibration_data[8]);
  cdata->dig_P3 = (int16_t)BME280_MAKE_WORD(calibration_data[11], calibration_data[10]);
  cdata->dig_P4 = (int16_t)BME280_MAKE_WORD(calibration_data[13], calibration_data[12]);
  cdata->dig_P5 = (int16_t)BME280_MAKE_WORD(calibration_data[15], calibration_data[14]);
  cdata->dig_P6 = (int16_t)BME280_MAKE_WORD(calibration_data[17], calibration_data[16]);
  cdata->dig_P7 = (int16_t)BME280_MAKE_WORD(calibration_data[19], calibration_data[18]);
  cdata->dig_P8 = (int16_t)BME280_MAKE_WORD(calibration_data[21], calibration_data[20]);
  cdata->dig_P9 = (int16_t)BME280_MAKE_WORD(calibration_data[23], calibration_data[22]);
  cdata->dig_H1 = (uint8_t)calibration_data[25];

  // raz array
  memset(calibration_data, 0, BME280_T_P_CALIB_DATA_LEN);

  // get values from register 0xE1 to 0xE7
  ret = bme280_read_register(BME280_REG_H_CALIB_DATA, BME280_H_CALIB_DATA_LEN, calibration_data);
  if (!ret)
    return 0;

  cdata->dig_H2 = (int16_t)BME280_MAKE_WORD(calibration_data[1], calibration_data[0]);
  cdata->dig_H3 = (uint8_t)calibration_data[2];
  cdata->dig_H4 = (int16_t)(int8_t)calibration_data[3] << 4 | (int16_t)(calibration_data[4] & 0x0F);
  cdata->dig_H5 = (int16_t)(int8_t)calibration_data[5] << 4 | (int16_t)(calibration_data[4] >> 4);
  cdata->dig_H6 = (int8_t)calibration_data[6];

  return 1;
}

uint8_t
bme280_read_sensor(BME280* bme)
{
  // if "forced mode" is enabled
  uint8_t sensor_mode = BME280_CTRL_MEAS & 0x03;
  if ((sensor_mode == 0x01) || (sensor_mode == 0x02))
  {
    // DS 3.3.3
    // for a next measurement, forced mode needs to be selected again
    bme280_write_register(BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS);
    // wait to ensure complete measurement
    delay(50);
  }

  uint8_t sensor_data[BME280_P_T_H_DATA_LEN] = { 0 };
  // get adc values from register 0xF7 to 0xFE
  uint8_t ret = bme280_read_register(BME280_REG_P_T_H_DATA, BME280_P_T_H_DATA_LEN, sensor_data);
  if (!ret)
    return 0;

  // pressure data before compensation (adc_p)
  const uint32_t adc_p = (uint32_t)sensor_data[0] << 12 |
                         (uint32_t)sensor_data[1] << 4  |
                         (uint32_t)sensor_data[2] >> 4;
  // temperature data before compensation (adc_t)
  const uint32_t adc_t = (uint32_t)sensor_data[3] << 12 |
                         (uint32_t)sensor_data[4] << 4  |
                         (uint32_t)sensor_data[5] >> 4;
  // humidity data before compensation (adc_h)
  const uint32_t adc_h = (uint32_t)sensor_data[6] << 8  |
                         (uint32_t)sensor_data[7];

  bme->temperature = 0;
  bme->pressure    = 0;
  bme->humidity    = 0;
  int32_t t_fine;
  struct calibration_data* cdata = &bme->calib_data;

  // if temperature is enabled, do compensation
  // see DS 4.2.3
  // Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
  if (adc_t != 0x80000)
  {
    int32_t var1, var2;
    var1 = ((((adc_t >> 3) - ((int32_t)cdata->dig_T1 << 1))) * ((int32_t)cdata->dig_T2)) >> 11;
    var2 = (((((adc_t >> 4) - ((int32_t)cdata->dig_T1)) * ((adc_t >> 4) - ((int32_t)cdata->dig_T1))) >> 12) * ((int32_t)cdata->dig_T3)) >> 14;
    t_fine = var1 + var2;
    bme->temperature = ((t_fine * 5 + 128) >> 8) + (ADJUST_TEMPERATURE_C * 100);
  }
  else
    return 0; // we can't compensate others measurements without t_fine define by temperature

  // if pressure is enabled, do compensation
  // see DS 4.2:
  // For 8-bit micro controllers, the variable size may be limited. In this case a simplified 32 bit integer
  // code with reduced accuracy is given in appendix 8.2.
  // -> DS Annexe 8.2
  // Returns pressure in Pa as unsigned 32 bit integer. Output value of "96386" equals 96386 Pa = 963.86 hPa
  if (adc_p != 0x80000)
  {
    int32_t  var1, var2;
    uint32_t p;

    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)cdata->dig_P6);
    var2 = var2 + ((var1 * ((int32_t)cdata->dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)cdata->dig_P4) << 16);
    var1 = (((cdata->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)cdata->dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)cdata->dig_P1)) >> 15);
    /* avoid exception caused by division by zero */
    if (var1) {
      p = (((uint32_t)(((int32_t)1048576) - adc_p) - (var2 >> 12))) * 3125;
      if (p < 0x80000000)
        p = (p << 1) / ((uint32_t)var1);
      else
        p = (p / (uint32_t)var1) * 2;
      var1 = (((int32_t)cdata->dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
      var2 = (((int32_t)(p >> 2)) * ((int32_t)cdata->dig_P8)) >> 13;
      p = (uint32_t)((int32_t)p + ((var1 + var2 + cdata->dig_P7) >> 4));
      bme->pressure = p;
    }
  }

  // if humidity is enabled, do compensation
  // see DS 4.2.3
  if (adc_h != 0x80000)
  {
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_h << 14) - (((int32_t)cdata->dig_H4) << 20) - (((int32_t)cdata->dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)cdata->dig_H6)) >> 10) * (((v_x1_u32r *
                       ((int32_t)cdata->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                       ((int32_t)cdata->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)cdata->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    bme->humidity = (uint32_t)(v_x1_u32r >> 12);
  }

  return 1;
}

uint8_t
bme280_init(BME280* bme)
{
  Wire.begin();
  
  // check the chip id
  uint8_t chip_id;
  if (bme280_read_register(BME280_REG_CHIP_ID, 1, &chip_id) && chip_id != 0x60)
    return 0;
    
  if (!bme280_get_calibration_data(bme))
    return 0;

  // DS 3.3
  // sleep mode: all registers accessible, selected after startup
  // so, we soft reset to put the chip in sleep mode
  // and safely access all registers to write our configuration
  bme280_write_register(BME280_REG_RESET, BME280_RESET_CMD);
  
  // DS 1.1
  // start-up time : max 2 ms (take 5)
  delay(5);

  // init oversampling and mode
  bme280_write_register(BME280_REG_CTRL_HUM,  BME280_CTRL_HUM);
  bme280_write_register(BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS);
  
  // init config (t_sb & filter)
  bme280_write_register(BME280_REG_CTRL_CONFIG, BME280_CTRL_CONFIG);
  
  return 1;
}
