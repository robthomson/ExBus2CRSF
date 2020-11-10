#include <stdint.h> 
#include <limits.h>
#include "RTClib.h"
#include <Time.h>
#include "xfire.h"
#include <inttypes.h>
#include "JetiExBusProtocol.h"
#include "ExbusSensor.h"


uint32_t lastRefreshTime;
uint8_t frame[CROSSFIRE_FRAME_MAXLEN];
extern JetiExBusProtocol exBus;

uint8_t telemetryRxBuffer[TELEMETRY_RX_PACKET_SIZE];
uint8_t telemetryRxBufferCount=0;



const CrossfireSensor crossfireSensors[] = {
  {LINK_ID,        0, ZSTR_RX_RSSI1,      UNIT_DB,                0},
  {LINK_ID,        1, ZSTR_RX_RSSI2,      UNIT_DB,                0},
  {LINK_ID,        2, ZSTR_RX_QUALITY,    UNIT_PERCENT,           0},
  {LINK_ID,        3, ZSTR_RX_SNR,        UNIT_DB,                0},
  {LINK_ID,        4, ZSTR_ANTENNA,       UNIT_RAW,               0},
  {LINK_ID,        5, ZSTR_RF_MODE,       UNIT_RAW,               0},
  {LINK_ID,        6, ZSTR_TX_POWER,      UNIT_MILLIWATTS,        0},
  {LINK_ID,        7, ZSTR_TX_RSSI,       UNIT_DB,                0},
  {LINK_ID,        8, ZSTR_TX_QUALITY,    UNIT_PERCENT,           0},
  {LINK_ID,        9, ZSTR_TX_SNR,        UNIT_DB,                0},
  {BATTERY_ID,     0, ZSTR_BATT,          UNIT_VOLTS,             1},
  {BATTERY_ID,     1, ZSTR_CURR,          UNIT_AMPS,              1},
  {BATTERY_ID,     2, ZSTR_CAPACITY,      UNIT_MAH,               0},
  {BATTERY_ID,     3, ZSTR_BATT_PERCENT,  UNIT_PERCENT,           0},
  {GPS_ID,         0, ZSTR_GPS,           UNIT_GPS_LATITUDE,      0},
  {GPS_ID,         0, ZSTR_GPS,           UNIT_GPS_LONGITUDE,     0},
  {GPS_ID,         2, ZSTR_GSPD,          UNIT_KMH,               1},
  {GPS_ID,         3, ZSTR_HDG,           UNIT_DEGREE,            3},
  {GPS_ID,         4, ZSTR_ALT,           UNIT_METERS,            0},
  {GPS_ID,         5, ZSTR_SATELLITES,    UNIT_RAW,               0},
  {ATTITUDE_ID,    0, ZSTR_PITCH,         UNIT_RADIANS,           3},
  {ATTITUDE_ID,    1, ZSTR_ROLL,          UNIT_RADIANS,           3},
  {ATTITUDE_ID,    2, ZSTR_YAW,           UNIT_RADIANS,           3},
  {FLIGHT_MODE_ID, 0, ZSTR_FLIGHT_MODE,   UNIT_TEXT,              0},
  {CF_VARIO_ID,    0, ZSTR_VSPD,          UNIT_METERS_PER_SECOND, 2},
  {0,              0, "UNKNOWN",          UNIT_RAW,               0},
};

const CrossfireSensor & getCrossfireSensor(uint8_t id, uint8_t subId)
{
  if (id == LINK_ID)
    return crossfireSensors[RX_RSSI1_INDEX+subId];
  else if (id == BATTERY_ID)
    return crossfireSensors[BATT_VOLTAGE_INDEX+subId];
  else if (id == GPS_ID)
    return crossfireSensors[GPS_LATITUDE_INDEX+subId];
  else if (id == CF_VARIO_ID)
    return crossfireSensors[VERTICAL_SPEED_INDEX];
  else if (id == ATTITUDE_ID)
    return crossfireSensors[ATTITUDE_PITCH_INDEX+subId];
  else if (id == FLIGHT_MODE_ID)
    return crossfireSensors[FLIGHT_MODE_INDEX];
  else
    return crossfireSensors[UNKNOWN_INDEX];
}


void runCrossfireTelemetry(){

      /*
      if(CROSSFIRE_SERIAL.available()){
        telemetryRxBufferCount = CROSSFIRE_SERIAL.available();
        processCrossfireTelemetryData(CROSSFIRE_SERIAL.read());
     }
     */
      telemetryRxBufferCount = CROSSFIRE_SERIAL.available();
      for (int i = 0; i < telemetryRxBufferCount; i++) {
        processCrossfireTelemetryData(CROSSFIRE_SERIAL.read());
      }     
   
}


void startCrossfire(){
     CROSSFIRE_SERIAL.begin(CROSSFIRE_BAUD_RATE,SERIAL_8N1_RXINV_TXINV);
}

void runCrossfire()
{
        memset(frame, 0, sizeof(frame));
        uint8_t length = createCrossfireChannelsFrame(frame);
  
        CROSSFIRE_SERIAL.write(frame, length); 



}

template<int N>
bool getCrossfireTelemetryValue(uint8_t index, int32_t & value)
{
  bool result = false;
  uint8_t * byte = &telemetryRxBuffer[index];
  value = (*byte & 0x80) ? -1 : 0;
  for (uint8_t i=0; i<N; i++) {
    value <<= 8;
    if (*byte != 0xff) {
      result = true;
    }
    value += *byte++;
  }
  return result;
}



void processCrossfireTelemetryData(uint8_t data)
{


  
  if (telemetryRxBufferCount == 0 && data != RADIO_ADDRESS) {
    Serial.print("[XF] address 0x%02X error ");
    Serial.println(data);
    return;
  }

  if (telemetryRxBufferCount == 1 && (data < 2 || data > TELEMETRY_RX_PACKET_SIZE-2)) {
    Serial.print("[XF] length 0x%02X error ");
    Serial.println(data);
        
    telemetryRxBufferCount = 0;
    return;
  }

  if (telemetryRxBufferCount < TELEMETRY_RX_PACKET_SIZE) {
    telemetryRxBuffer[telemetryRxBufferCount++] = data;
  }
  else {
    Serial.print("[XF] array size error ");
    Serial.println(telemetryRxBufferCount);    
    telemetryRxBufferCount = 0;
  }



  
  if (telemetryRxBufferCount > 4) {
    uint8_t length = telemetryRxBuffer[1];

  // Serial.println("Reached step before Process Frame");    
    if (length + 2 == telemetryRxBufferCount) {
      processCrossfireTelemetryFrame();
      telemetryRxBufferCount = 0;
   }
  }

  
}

void processCrossfireTelemetryFrame(){
   // Serial.println("Reached Process Frame");
    if (!checkCrossfireTelemetryFrameCRC()) {
  //  Serial.println("[XF] CRC error");
    return;
    }
Serial.println("Reached Step After Process Frame");
  uint8_t id = telemetryRxBuffer[2];
  int32_t value;

  Serial.print("ID: ");
  Serial.print(id);
  Serial.print(" VALUE: ");  
  Serial.println(value);  

  
  switch(id) {
    case CF_VARIO_ID:
      if (getCrossfireTelemetryValue<2>(3, value))
        processCrossfireTelemetryValue(VERTICAL_SPEED_INDEX, value);
      break;


  }

  
}



void processCrossfireTelemetryValue(uint8_t index, int32_t value)
{
  const CrossfireSensor & sensor = crossfireSensors[index];
  //setTelemetryValue(PROTOCOL_TELEMETRY_CROSSFIRE, sensor.id, 0, sensor.subId, value, sensor.unit, sensor.precision);
  Serial.println(value);
}


uint8_t createCrossfireChannelsFrame(uint8_t * frame)
{
  uint8_t * buf = frame;
  *buf++ = MODULE_ADDRESS;
  *buf++ = 24; // 1(ID) + 22 + 1(CRC)
  uint8_t * crc_start = buf;
  *buf++ = CHANNELS_ID;
  uint32_t bits = 0;
  uint8_t bitsavailable = 0;


              for (int i=0; i<CROSSFIRE_CHANNELS_COUNT; i++) {
            
                //uint32_t val = CROSSFIRE_CENTER;
                uint32_t val = map(exBus.GetChannel(i),EXBUS_LOW,EXBUS_HIGH,CROSSFIRE_LOW,CROSSFIRE_HIGH);
            
                bits |= val << bitsavailable;
                bitsavailable += CROSSFIRE_CH_BITS;
                while (bitsavailable >= 8) {
                  *buf++ = bits;
                  bits >>= 8;
                  bitsavailable -= 8;
                }
              }
              *buf++ = crc8(crc_start, 23);

              return buf - frame;
            }

       

// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
unsigned char crc8tab[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
  0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
  0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
  0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
  0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
  0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
  0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
  0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
  0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
  0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
  0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
  0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
  0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
  0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
  0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
  0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
  0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crc8(const uint8_t * ptr, uint32_t len)
{
  uint8_t crc = 0;
  for ( uint32_t i=0 ; i<len ; i += 1 )
  {
    crc = crc8tab[crc ^ *ptr++] ;
  }
  return crc;
}


bool checkCrossfireTelemetryFrameCRC()
{
  uint8_t len = telemetryRxBuffer[1];
  uint8_t crc = crc8(&telemetryRxBuffer[2], len-1);
  return (crc == telemetryRxBuffer[len+1]);
}
