#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

/********************************************/
/************* USER CONFIGURATION ***********/
/********************************************/

/*  
 *   wiring (ATtiny85):
 *                             +-------+
 *                            1|*      |8  VCC
 *                            2|       |7  SCL (BME280)
 *   Data TX RF 433Mhz - PB4  3|       |6
 *                       GND  4|       |5  SDA (BME280)
 *                             +-------+
 *
 *   Warning:
 *     - BME280 MAX supported voltage is 3.6v (DS 1 Table 1.1)
 *     
*/
  
/***
 * USAGE
 */

/* 
 * Choose your mode
 * 0 : Temperature                       [THN132N]
 * 1 : Temperature + Humidity            [THGR122N]
 * 2 : Temperature + Humidity + Pressure [BHTR968]
*/ 
 
#define MODE                    (2)

/* Roughly, how many SECONDS should I sleep between 2 transmissions ? */

#define SLEEPING_TIME_S         (20)

/* 
 * Should I ALWAYS send messages or ONLY NEW message ? 
 * 0 : Always send messages
 *      - even if the new message is the same as the last one (no data has changed)
 * 1 : Only send if there is a NEW message
 *      - only send a message if at least a data has changed since the last message
 *        (battery level alert, temperature, humidity or pressure in regard of the chosen MODE) 
*/

#define SEND_ONLY_NEW_MESSAGE   (0)

/*
 * Should I send a message twice ?
 * A real Oregon Scientific sensor sends a message twice but if you have a good RF reception, 
 * imho, sending a message twice is a waste of energy.
 * 0 : Don't send a message twice
 * 1 : Send a message twice
*/

#define SEND_MESSAGE_TWICE      (1)

/* Limit in mV for LOW Battery Alert */

#define LOW_BATTERY_ALERT       (2750)

/* Oregon Scientific protocol */

/* CHANNEL
 * You could safely keep this as is
*/

#define CHANNEL                 (0x20)

/* ID
 * Each sensor must have a unique ID
 * eg:
 * - first sensor:  0xCC
 * - second sensor: 0xCD
 * - third sensor:  0xCE
 * - fourth sensor: 0xCF
 * - fifth sensor:  0xD0
 * - etc...
*/

#define ID                      (0xCC)


/***
 * BME280 PARAMETERS
 */
 
/* 
 * I2C address of the device
 * If your BME280 doesn't work with this address, try 0x77
*/

#define BME280_I2C_ADDR         (0x76)

/* If needed, you can adjust the temperature in °C returned by the sensor */

#define ADJUST_TEMPERATURE_C    (-0.7)

/* BME280 Configuration (oversampling, filter & mode)
 * This should be fine for most usage in our Low Energy quest.
 * Choices are:
 *  - Forced mode (take a measurement and the chip returns to sleep)
 *  - No filter
 *  - Oversammpling x1 for temperature, humidity and pressure
 * These choices are those recommended by Bosch Sensortec in their datasheet
 * for our use (weather station: minimal power consumption)
 * Some brief explanations are below if you want to modify these default settings
*/

#define BME280_CTRL_CONFIG      (0b00000000)
#if MODE == 0
#define BME280_CTRL_HUM         (0b00000000)
#define BME280_CTRL_MEAS        (0b00100001)
#elif MODE == 1
#define BME280_CTRL_HUM         (0b00000001)
#define BME280_CTRL_MEAS        (0b00100001)
#else
#define BME280_CTRL_HUM         (0b00000001)
#define BME280_CTRL_MEAS        (0b00100101)
#endif

/********************************************/
/********* END USER CONFIGURATION ***********/
/********************************************/

/* What is BME280_CTRL_CONFIG ?
 * --------------------------
 * CONFIG (RATE - FILTER - INTERFACE OPTIONS OF THE DEVICE)   
 * cf DS 5.4.6
 *
 * t_sb : STANDBY TIME IN NORMAL MODE
 * 000 :    0.5 ms
 * 110 :   10   ms
 * 111 :   20   ms
 * 001 :   62.5 ms
 * 010 :  125   ms
 * 011 :  250   ms
 * 100 :  500   ms
 * 101 : 1000   ms
 *
 * filter : FILTER COEFFICIENT
 * 000 : off
 * 001 :  2
 * 010 :  4
 * 011 :  8
 * 100 : 16
 *
 * config : Register 0xF5
 * byte = t_sb[2:0] : filter[2:0] : not used[0] : spi3w_en[0]
 *
 * #define BME280_CTRL_CONFIG  0b00000000
 *                               ------ -
 *                                 |  | |
 *                                 |  | -> spi3w_en (1 bit)
 *                                 |  ---> Filter coefficient (3 bits)
 *                                 ------> Stanby time in Normal Mode (t_sb: 3 bits)
 * 
 * 
 *
 * What is BME280_CTRL_HUM ?
 * -----------------------
 * HUMIDITY (osrs_h)
 * cf DS 5.4.3
 *  
 * OVERSAMPLING (humidity)
 * 000 : skipped
 * 001 : oversampling x1
 * 010 : oversampling x2
 * 011 : oversampling x4
 * 100 : oversampling x8
 * 101 : oversampling x16
 * 
 * ctrl_hum : Register 0xF2
 * byte = empty[4:0] : osrs_h[2:0]
 * note: changes to this register only become effective after a WRITE operation to ctrl_meas
 *
 * #define BME280_CTRL_HUM  0b00000001
 *                                 ---
 *                                   |
 *                                   -> Humidity oversampling (osrs_h: 3 bits)
 *                                   
 *
 *
 *
 * What is BME280_CTRL_MEAS ?
 * ------------------------
 * TEMPERATURE - PRESSION - MODE (osrs_t, osrs_p, mode)
 * cf DS 5.4.5
 *
 * OVERSAMPLING (temperature / pression)
 * 000 : skipped
 * 001 : oversampling x1
 * 010 : oversampling x2
 * 011 : oversampling x4
 * 100 : oversampling x8
 * 101 : oversampling x16
 * 
 * DEVICE MODE
 * 00       : sleep
 * 01 or 10 : forced
 * 11       : normal
 *
 * ctrl_meas : Register 0xF4
 * byte = osrs_t[2:0] : osrs_p[2:0] : mode[1:0]
 *
 * #define BME280_CTRL_MEAS  0b00100101
 *                             --------
 *                               T  P M
 *                               |  | |
 *                               |  | -> Mode (2 bits)
 *                               |  ---> Pressure oversampling (osrs_p: 3 bits)
 *                               ------> Temperature oversampling (osrs_t: 3 bits)
*/ 

#endif
