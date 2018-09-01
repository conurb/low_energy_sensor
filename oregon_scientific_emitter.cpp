/*
 * connectingStuff, Oregon Scientific v2.1 Emitter
 * http://www.connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/
 *
 * Copyright (C) 2013 olivier.lebrun@gmail.com
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
 * Slightly modified by conurb (09/2018) : 
 * - split code in .cpp & .h files
 * - simpler api usage in code
 * - some renaming
 * - a very little byte hunting (only 8kb available on an ATtiny85)
 * 
*/

#include "oregon_scientific_emitter.h"

byte OregonMessageBuffer[MESSAGE_BUF_LEN];

/**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendZero(void) 
{
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}

/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendOne(void) 
{
   SEND_LOW();
   delayMicroseconds(TIME);
   SEND_HIGH();
   delayMicroseconds(TWOTIME);
   SEND_LOW();
   delayMicroseconds(TIME);
}
 
/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/
 
/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterMSB(const byte data) 
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}
 
/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterLSB(const byte data) 
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/
 
/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void sendData(byte* data, byte size)
{
  for(byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}

/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void sendPreamble(void)
{
  byte PREAMBLE[]={0xFF,0xFF};
  sendData(PREAMBLE, 2);
}

/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void sendPostamble(void)
{
#if MODE == 0
  sendQuarterLSB(0x00);
#else
  byte POSTAMBLE[]={0x00};
  sendData(POSTAMBLE, 1);
#endif
}

/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Sum(byte count, const byte* data)
{
  int s = 0;
 
  for(byte i = 0; i<count;i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }
 
  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;
 
  return s;
}

/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
void calculateAndSetChecksum(byte* data)
{
#if MODE == 0
    int s = ((Sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);
    data[6] |=  (s&0x0F) << 4;
    data[7] =   (s&0xF0) >> 4;
#elif MODE == 1
    data[8] = ((Sum(8, data) - 0xa) & 0xFF);
#else
    data[10] = ((Sum(10, data) - 0xa) & 0xFF);
#endif
}
 
/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/

/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void oregon_send_message(byte *data, byte size)
{
    calculateAndSetChecksum(data);
    sendPreamble();
    //sendSync();
    sendData(data, size);
    sendPostamble();
}

/**
 * \brief    Set the sensor battery level
 * \param    level      Battery level (0 = low, 1 = high)
 * \param    data       Oregon message
 */
void oregon_set_battery_level(byte level, byte *data)
{
  if(!level) data[4] = 0x0C;
  else data[4] = 0x00;
}

/**
 * \brief    Set the sensor temperature
 * \param    temp       the temperature
 * \param    data       Oregon message
 */
void oregon_set_temperature(float temp, byte *data) 
{
  // Set temperature sign
  if(temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;  
  }
  else
  {
    data[6] = 0x00;
  }
 
  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);
 
  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);
 
  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;
 
  // Set temperature float part
  data[4] |= (tempFloat << 4);
}

/**
 * \brief    Set the sensor humidity
 * \param    hum        the humidity
 * \param    data       Oregon message
 */
void oregon_set_humidity(byte hum, byte* data)
{ 
    data[7] = (hum/10);
    data[6] |= (hum - data[7]*10) << 4;
}

/**
 * \brief    Set the sensor pressure
 * \param    pres       the pressure
 * \param    data       Oregon message
 */
void oregon_set_pressure(float pres, byte *data) 
{
  if ((pres > 850) && (pres < 1100)) {
    data[9] = 0xC0;
    data[8] = (int)lrintf(pres) - 856;  
  }
}

/**
 * \brief    Set the sensor type
 * \param    type       Sensor type
 * \param    data       Oregon message
 */
static inline void oregon_set_type(byte* type, byte *data = OregonMessageBuffer) 
{
  data[0] = type[0];
  data[1] = type[1];
}

/**
 * \brief    Set the sensor channel
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 * \param    data       Oregon message
 */
static inline void oregon_set_channel(byte channel, byte *data = OregonMessageBuffer) 
{
    data[2] = channel;
}

/**
 * \brief    Set the sensor ID
 * \param    ID         Sensor unique ID
 * \param    data       Oregon message
 */
static inline void oregon_set_id(byte id, byte *data = OregonMessageBuffer) 
{
  data[3] = id;
}

/**
 * \brief    Init the sensor
 */
void oregon_init()
{
#if MODE == 0
byte TYPE[] = {0xEA, 0x4C};
#elif MODE == 1
byte TYPE[] = {0x1A, 0x2D};
#else
byte TYPE[] = {0x5A, 0x6D};
#endif
#ifndef CHANNEL
#define CHANNEL (0x20)
#endif
#ifndef ID
#define ID (0xCC)
#endif
  oregon_set_type(TYPE);
  oregon_set_channel(CHANNEL);
  oregon_set_id(ID);
}

/******************************************************************/
/******************************************************************/
/******************************************************************/
