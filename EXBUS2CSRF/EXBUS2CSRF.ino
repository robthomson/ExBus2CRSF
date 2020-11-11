

#include "JetiExBusProtocol.h"
#include "ExbusSensor.h"
#include <stdint.h> 
#include "xfire.h"
#include <limits.h>
#include "RTClib.h"
#include <Time.h>
#include <inttypes.h>
#include "TeensyTimerTool.h"


using namespace TeensyTimerTool;
//PeriodicTimer csrfTimer;


JetiExBusProtocol exBus;
ExbusSensor        exbusSensor; 


float sensorGPSLat;
float sensorGPSLong;
float sensorHeading=0;
float sensorAltitude=0;
uint32_t sensorSpeed=0;
uint32_t sensorSats=0;
float sensorPitch=0;
float sensorRoll=0;
float sensorYaw=0;
double sensorVoltage;
double sensorCurrent;
double sensorFuel;
float sensorVario;
double sensorRSSI;
double sensorSNR;
double sensorTXPWR;

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
	/*{ ID_VOLTAGE,    "Voltage",    "V",         JetiSensor::TYPE_14b, 1 },*/
	/*{ ID_ALTITUDE,   "Altitude",   "m",         JetiSensor::TYPE_14b, 0 },*/
	/*{ ID_TEMP,       "Temp",       "\xB0\x43",  JetiSensor::TYPE_14b, 0 }, // �C*/
/*	{ ID_CLIMB,      "Climb",      "m/s",       JetiSensor::TYPE_14b, 2 },*/
	/*{ ID_FUEL,       "Fuel",       "%",         JetiSensor::TYPE_14b, 0 },*/
	/*{ ID_RPM,        "RPM x 1000", "/min",      JetiSensor::TYPE_14b, 1 },*/

	{ ID_GPSLON,     "Longitude",  " ",         JetiSensor::TYPE_GPS, 0 },
	{ ID_GPSLAT,     "Latitude",   " ",         JetiSensor::TYPE_GPS, 0 },
	/*{ ID_DATE,       "Date",       " ",         JetiSensor::TYPE_DT,  0 },*/
	/*{ ID_TIME,       "Time",       " ",         JetiSensor::TYPE_DT,  0 },*/

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


 runCrossfire();

    
/*
	// get JETI buttons - at some point maybe make this all work via jetibox?
	uint8_t bt = exBus.GetJetiboxKey();
	if( bt )
	{
		Serial.print( "bt - "); Serial.println(bt);
    }

	if (exBus.IsBusReleased())
	{
		// exBus is currently sending an ex packet
		// do time consuming stuff here (20-30ms)
		delay( 30 );
	}
*/



 // exBus.SetSensorValue(ID_VOLTAGE, sensorVoltage);
  
 // exBus.SetSensorValue(ID_ALTITUDE, sensorAltitude);

  
	exBus.SetSensorValue(ID_TEMP, exbusSensor.GetTemp());
	exBus.SetSensorValue(ID_CLIMB, exbusSensor.GetClimb());
	//exBus.SetSensorValue(ID_FUEL, exbusSensor.GetFuel());
	exBus.SetSensorValue(ID_RPM, exbusSensor.GetRpm());

	//exBus.SetSensorValueGPS(ID_GPSLON, true, 11.55616f); // E 11� 33' 22.176"
	//exBus.SetSensorValueGPS(ID_GPSLAT, false, 48.24570f); // N 48� 14' 44.520"
   exBus.SetSensorValueGPS(ID_GPSLAT, false, sensorGPSLat); 
   exBus.SetSensorValueGPS(ID_GPSLON, true, sensorGPSLong); 

  /*
	exBus.SetSensorValueDate(ID_DATE, 29, 12, 2015);
	exBus.SetSensorValueTime(ID_TIME, 19, 16, 37);
*/
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
  
  /*


	exBus.SetSensorValue(ID_VAL15, exbusSensor.GetVal(8));
	exBus.SetSensorValue(ID_VAL16, exbusSensor.GetVal(9));
	exBus.SetSensorValue(ID_VAL17, exbusSensor.GetVal(10));
	exBus.SetSensorValue(ID_VAL18, exbusSensor.GetVal(11));
*/

	exBus.DoJetiExBus();

 


}
