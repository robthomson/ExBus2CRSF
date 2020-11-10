

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
	ID_VAL11, ID_VAL12, ID_VAL13, ID_VAL14, ID_VAL15, ID_VAL16, ID_VAL17, ID_VAL18,
};


//csrf vars



// sensor definition (max. 31 for DC/DS-16)
// name plus unit must be < 20 characters
// precision = 0 --> 0, precision = 1 --> 0.0, precision = 2 --> 0.00

JETISENSOR_CONST sensors[] PROGMEM =
{
	// id             name          unit         data type             precision 
	{ ID_VOLTAGE,    "Voltage",    "V",         JetiSensor::TYPE_14b, 1 },
	{ ID_ALTITUDE,   "Altitude",   "m",         JetiSensor::TYPE_14b, 0 },
	{ ID_TEMP,       "Temp",       "\xB0\x43",  JetiSensor::TYPE_14b, 0 }, // �C
	{ ID_CLIMB,      "Climb",      "m/s",       JetiSensor::TYPE_14b, 2 },
	{ ID_FUEL,       "Fuel",       "%",         JetiSensor::TYPE_14b, 0 },
	{ ID_RPM,        "RPM x 1000", "/min",      JetiSensor::TYPE_14b, 1 },

	{ ID_GPSLON,     "Longitude",  " ",         JetiSensor::TYPE_GPS, 0 },
	{ ID_GPSLAT,     "Latitude",   " ",         JetiSensor::TYPE_GPS, 0 },
	{ ID_DATE,       "Date",       " ",         JetiSensor::TYPE_DT,  0 },
	{ ID_TIME,       "Time",       " ",         JetiSensor::TYPE_DT,  0 },

	{ ID_VAL11,      "V11",        "U11",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL12,      "V12",        "U12",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL13,      "V13",        "U13",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL14,      "V14",        "U14",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL15,      "V15",        "U15",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL16,      "V16",        "U16",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL17,      "V17",        "U17",       JetiSensor::TYPE_14b, 0 },
	{ ID_VAL18,      "V18",        "U18",       JetiSensor::TYPE_14b, 0 },
	{ 0 } // end of array
};



void setup()
{
  Serial.begin(9600);

  startCrossfire();

  
  //csrfTimer.begin(runCrossfire, (REFRESH_INTERVAL*1000)); 

	exBus.SetDeviceId(0x76, 0x32); // 0x3276
	exBus.Start("EX Bus", sensors, 2 ); // com port: 1..3 for Teeny, 0 or 1 for AtMega328PB UART0/UART1, others: not used 

	exBus.SetJetiboxText(0, "EXBUS2CSRF");
	exBus.SetJetiboxText(1, "TRANSCODER");

}


void loop()
{


 runCrossfire();

    

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




  exBus.SetSensorValue(ID_VOLTAGE, exbusSensor.GetVoltage());
	exBus.SetSensorValue(ID_ALTITUDE, exbusSensor.GetAltitude());
	exBus.SetSensorValue(ID_TEMP, exbusSensor.GetTemp());
	exBus.SetSensorValue(ID_CLIMB, exbusSensor.GetClimb());
	exBus.SetSensorValue(ID_FUEL, exbusSensor.GetFuel());
	exBus.SetSensorValue(ID_RPM, exbusSensor.GetRpm());

	exBus.SetSensorValueGPS(ID_GPSLON, true, 11.55616f); // E 11� 33' 22.176"
	exBus.SetSensorValueGPS(ID_GPSLAT, false, 48.24570f); // N 48� 14' 44.520"
	exBus.SetSensorValueDate(ID_DATE, 29, 12, 2015);
	exBus.SetSensorValueTime(ID_TIME, 19, 16, 37);

	exBus.SetSensorValue(ID_VAL11, exbusSensor.GetVal(4));
	exBus.SetSensorValue(ID_VAL12, exbusSensor.GetVal(5));
	exBus.SetSensorValue(ID_VAL13, exbusSensor.GetVal(6));
	exBus.SetSensorValue(ID_VAL14, exbusSensor.GetVal(7));
	exBus.SetSensorValue(ID_VAL15, exbusSensor.GetVal(8));
	exBus.SetSensorValue(ID_VAL16, exbusSensor.GetVal(9));
	exBus.SetSensorValue(ID_VAL17, exbusSensor.GetVal(10));
	exBus.SetSensorValue(ID_VAL18, exbusSensor.GetVal(11));

	exBus.DoJetiExBus();

 


}
