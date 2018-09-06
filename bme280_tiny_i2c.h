#ifndef __BME280_TINY_I2C_H__
#define __BME280_TINY_I2C_H__

/*********** REGISTERS ADDRESSES ************/
#define BME280_REG_CTRL_HUM       (0xF2)
#define BME280_REG_CTRL_MEAS      (0xF4)
#define BME280_REG_CTRL_CONFIG    (0xF5)
#define BME280_REG_CHIP_ID        (0xD0)
#define BME280_REG_T_P_CALIB_DATA (0x88)
#define BME280_REG_H_CALIB_DATA   (0xE1)
#define BME280_REG_P_T_H_DATA     (0xF7)
#define BME280_REG_RESET          (0xE0)
/**************** COMMAND *******************/
#define BME280_RESET_CMD          (0x86)
/**************** LENGTH ********************/
#define BME280_T_P_CALIB_DATA_LEN (26)
#define BME280_H_CALIB_DATA_LEN	  (7)
#define BME280_P_T_H_DATA_LEN     (8)
/**************** MACROS ********************/
#define BME280_MAKE_WORD(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)
/*********** CALIBRATION DATAS **************/
struct calibration_data {
  // temperature
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;

  // pressure
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;

  // humidity
  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
};
/************* SENSOR VALUES ****************/
typedef struct {
  int32_t  temperature; // div by  100.0 = DegC
  uint32_t humidity;    // div by 1024.0 = % RH
  uint32_t pressure;    // div by  100.0 = hPa
  struct calibration_data calib_data;
} BME280;

/******************** API *******************/
uint8_t bme280_read_sensor(BME280*);
uint8_t bme280_init(BME280*);

#endif
