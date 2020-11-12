/*
  Copyright (C) Rob Thomson
 
  Based on code named fromn the opentx project
    th9x - http://code.google.com/p/th9x
    er9x - http://code.google.com/p/er9x
    gruvin9x - http://code.google.com/p/gruvin9x
 
  License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 
  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  Jeti  EX Bus C++ Library for Teensy 3.x
  -------------------------------------------------------------------
  
  Copyright (C) 2018 Bernd Wokoeck
  
  Version history:
  0.90   02/04/2018  created
  0.91   02/09/2018  Support for AtMega32u4 added
  0.92   02/14/2018  Support for ESP32 added
  0.93   02/16/2018  ESP32 uart initialization changed
  0.94   02/17/2018  Generic arduino HardwareSerial support for AtMega328PB
  0.95   03/17/2018  Synchronization (IsBusReleased) for time consuming operations
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  IN THE SOFTWARE.
**************************************************************/

 
#include "JetiExBusProtocol.h"
#include "ExbusSensor.h"
#include <stdint.h> 
#include "xfire.h"
#include <limits.h>
#include "RTClib.h"
#include <Time.h>
#include <inttypes.h>

JetiExBusProtocol exBus;
ExbusSensor        exbusSensor; 


//bring in all externals related to crossfire
extern uint32_t crossfireChannels[CROSSFIRE_CHANNELS_COUNT];  //pulses data
extern float sensorVario;
extern double sensorGPSLat;
extern double sensorGPSLong;
extern float sensorAltitude;
extern float sensorHeading;
extern uint32_t sensorSpeed;
extern uint32_t sensorSats;
extern float sensorPitch;
extern float sensorRoll;
extern float sensorYaw;
extern double sensorVoltage;
extern double sensorCurrent;
extern double sensorFuel;
extern double sensorRSSI;
extern double sensorSNR;
extern double sensorTXPWR;


enum
{
	ID_VOLTAGE = 1,
	ID_ALTITUDE,
	ID_TEMP,
	ID_CLIMB,
	ID_FUEL,
	ID_RPM,
	ID_GPSLON,
	ID_GPSLAT,
	ID_DATE,
	ID_TIME,
	ID_VAL11, ID_VAL12, ID_VAL13, ID_VAL14, ID_VAL15, ID_VAL16, ID_VAL17, ID_VAL18, ID_VAL19, ID_VAL20, ID_VAL21, ID_VAL22,ID_VAL23,ID_VAL24
};


//csrf vars



// sensor definition (max. 31 for DC/DS-16)
// name plus unit must be < 20 characters
// precision = 0 --> 0, precision = 1 --> 0.0, precision = 2 --> 0.00

JETISENSOR_CONST sensors[] PROGMEM =
{
	// id             name          unit         data type             precision 
	{ ID_GPSLON,     "Longitude",  " ",         JetiSensor::TYPE_GPS, 0 },
	{ ID_GPSLAT,     "Latitude",   " ",         JetiSensor::TYPE_GPS, 0 },
	{ ID_VAL11,      "Ground Speed",        "m/s",       JetiSensor::TYPE_14b, 2 },
	{ ID_VAL12,      "Altitude",        "m",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL13,      "Satellites",        "",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL14,      "Att. Pitch",        "rad",       JetiSensor::TYPE_14b, 2 },
  { ID_VAL15,      "Att. Roll",        "rad",       JetiSensor::TYPE_14b, 2 },
 	{ ID_VAL16,      "Att. Yaw",        "rad",       JetiSensor::TYPE_14b, 2 },
 	{ ID_VAL17,      "Heading",        "\xB0",       JetiSensor::TYPE_14b, 2 },
  { ID_VAL18,      "Voltage",        "V",       JetiSensor::TYPE_14b, 1 },
  { ID_VAL19,      "Current",        "A",       JetiSensor::TYPE_14b, 1 },
  { ID_VAL20,      "Fuel",        "%",       JetiSensor::TYPE_14b, 1 },
  { ID_VAL21,      "Vario",        "m/s",       JetiSensor::TYPE_14b, 2 },
  { ID_VAL22,      "RSSI",        "%",       JetiSensor::TYPE_14b, 0 },
  { ID_VAL23,      "SNR",        "%",       JetiSensor::TYPE_14b, 0 },
  { ID_VAL24,      "TXPWR",        "mw",       JetiSensor::TYPE_14b, 0 },
	{ 0 } // end of array
};



void setup()
{
  Serial.begin(9600);

  startCrossfire();

	exBus.SetDeviceId(0x76, 0x32); // 0x3276
	exBus.Start("EX Bus", sensors, 2 ); // com port: 1..3 for Teeny, 0 or 1 for AtMega328PB UART0/UART1, others: not used 

	exBus.SetJetiboxText(0, "EXBUS2CSRF");
	exBus.SetJetiboxText(1, "TRANSCODER");

}


void loop()
{

   //run crossfire telemetry and pulses
   runCrossfire();

   //run jeti telemetry and pulses
   if ( exBus.HasNewChannelData() )
  {
    int i;
    for (i = 0; i < exBus.GetNumChannels(); i++)
    {
        crossfireChannels[i] =  map(exBus.GetChannel(i),EXBUS_LOW,EXBUS_HIGH,CROSSFIRE_LOW,CROSSFIRE_HIGH);
    }
  }

   //set telemetry values
	 exBus.SetSensorValue(ID_TEMP, exbusSensor.GetTemp());
	 exBus.SetSensorValue(ID_CLIMB, exbusSensor.GetClimb());
	 exBus.SetSensorValue(ID_RPM, exbusSensor.GetRpm());
   exBus.SetSensorValueGPS(ID_GPSLAT, false, sensorGPSLat); 
   exBus.SetSensorValueGPS(ID_GPSLON, true, sensorGPSLong); 
   exBus.SetSensorValue(ID_VAL11, sensorSpeed);
   exBus.SetSensorValue(ID_VAL12, sensorAltitude);
   exBus.SetSensorValue(ID_VAL13, sensorSats); 
   exBus.SetSensorValue(ID_VAL14, sensorPitch);  
   exBus.SetSensorValue(ID_VAL15, sensorRoll);  
   exBus.SetSensorValue(ID_VAL16, sensorYaw);    
   exBus.SetSensorValue(ID_VAL17, sensorHeading);    
   exBus.SetSensorValue(ID_VAL18, sensorVoltage);
   exBus.SetSensorValue(ID_VAL19, sensorCurrent);
   exBus.SetSensorValue(ID_VAL20, sensorFuel);  
   exBus.SetSensorValue(ID_VAL21, sensorVario);  
   exBus.SetSensorValue(ID_VAL22, sensorRSSI);    
   exBus.SetSensorValue(ID_VAL23, sensorSNR);   
   exBus.SetSensorValue(ID_VAL24, sensorTXPWR);     

   exBus.DoJetiExBus();



}
