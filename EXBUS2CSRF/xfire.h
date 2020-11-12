/*
 * Copyright (C) Rob Thomson
 *
 * Based on code named fromn the opentx project
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define XFIRE_DEBUG                       0; 

#define REFRESH_INTERVAL 16  
#define CROSSFIRE_BAUD_RATE 115200 

// this will not work on teensy with telemery as uart cant handle speed.
//#define REFRESH_INTERVAL 4    
//#define CROSSFIRE_BAUD_RATE 400000 


uint8_t createCrossfireChannelsFrame(uint8_t * frame);
void startCrossfire();
void runCrossfire();
bool checkCrossfireTelemetryFrameCRC();
void  runCrossfireTelemetry();
void processCrossfireTelemetryData(uint8_t data);
void processCrossfireTelemetryFrame();
void processCrossfireTelemetryValue(uint8_t index, int32_t value);
void setTelemetryValue( uint16_t id, uint8_t subId, uint8_t index, uint8_t instance, int32_t value, uint32_t unit, uint32_t prec);

uint8_t crc8(const uint8_t * ptr, uint32_t len);

#define CROSSFIRE_CH_BITS           11
#define CROSSFIRE_CENTER            0x3E0  //992
#define CROSSFIRE_LOW               173
#define CROSSFIRE_HIGH              1815
#define CROSSFIRE_CENTER_CH_OFFSET(ch)            (0)
#define CROSSFIRE_CHANNELS_COUNT  16
#define MODULE_ADDRESS              0xEE
#define CHANNELS_ID                 0x16
#define CROSSFIRE_FRAME_MAXLEN         64
#define CROSSFIRE_SERIAL Serial1
#define EXBUS_LOW   8000
#define EXBUS_HIGH  16000
#define TELEMETRY_RX_PACKET_SIZE       128
#define RADIO_ADDRESS                  0xEA


enum TelemetryUnit {
  UNIT_RAW,
  UNIT_VOLTS,
  UNIT_AMPS,
  UNIT_MILLIAMPS,
  UNIT_KTS,
  UNIT_METERS_PER_SECOND,
  UNIT_FEET_PER_SECOND,
  UNIT_KMH,
  UNIT_SPEED = UNIT_KMH,
  UNIT_MPH,
  UNIT_METERS,
  UNIT_DIST = UNIT_METERS,
  UNIT_FEET,
  UNIT_CELSIUS,
  UNIT_TEMPERATURE = UNIT_CELSIUS,
  UNIT_FAHRENHEIT,
  UNIT_PERCENT,
  UNIT_MAH,
  UNIT_WATTS,
  UNIT_MILLIWATTS,
  UNIT_DB,
  UNIT_RPMS,
  UNIT_G,
  UNIT_DEGREE,
  UNIT_RADIANS,
  UNIT_MILLILITERS,
  UNIT_FLOZ,
  UNIT_MILLILITERS_PER_MINUTE,
  UNIT_HERTZ,
  UNIT_MS,
  UNIT_US,
  UNIT_MAX = UNIT_US,
  UNIT_SPARE4,
  UNIT_SPARE5,
  UNIT_SPARE6,
  UNIT_SPARE7,
  UNIT_SPARE8,
  UNIT_SPARE9,
  UNIT_SPARE10,
  UNIT_HOURS,
  UNIT_MINUTES,
  UNIT_SECONDS,
  // FrSky format used for these fields, could be another format in the future
  UNIT_FIRST_VIRTUAL,
  UNIT_CELLS = UNIT_FIRST_VIRTUAL,
  UNIT_DATETIME,
  UNIT_GPS,
  UNIT_BITFIELD,
  UNIT_TEXT,
  // Internal units (not stored in sensor unit)
  UNIT_GPS_LONGITUDE,
  UNIT_GPS_LATITUDE,
  UNIT_DATETIME_YEAR,
  UNIT_DATETIME_DAY_MONTH,
  UNIT_DATETIME_HOUR_MIN,
  UNIT_DATETIME_SEC
};

#define ZSTR_VFR                       "VFR"
#define ZSTR_RSSI                      "RSSI"
#define ZSTR_R9PW                      "R9PW"
#define ZSTR_RAS                       "SWR"
#define ZSTR_A1                        "A1"
#define ZSTR_A2                        "A2"
#define ZSTR_A3                        "A3"
#define ZSTR_A4                        "A4"
#define ZSTR_BATT                      "RxBt"
#define ZSTR_ALT                       "Alt"
#define ZSTR_TEMP1                     "Tmp1"
#define ZSTR_TEMP2                     "Tmp2"
#define ZSTR_TEMP3                     "Tmp3"
#define ZSTR_TEMP4                     "Tmp4"
#define ZSTR_RPM2                      "RPM2"
#define ZSTR_PRES                      "Pres"
#define ZSTR_ODO1                      "Odo1"
#define ZSTR_ODO2                      "Odo2"
#define ZSTR_TXV                       "TX_V"
#define ZSTR_CURR_SERVO1               "CSv1"
#define ZSTR_CURR_SERVO2               "CSv2"
#define ZSTR_CURR_SERVO3               "CSv3"
#define ZSTR_CURR_SERVO4               "CSv4"
#define ZSTR_DIST                      "Dist"
#define ZSTR_ARM                       "Arm"
#define ZSTR_C50                       "C50"
#define ZSTR_C200                      "C200"
#define ZSTR_RPM                       "RPM"
#define ZSTR_FUEL                      "Fuel"
#define ZSTR_VSPD                      "VSpd"
#define ZSTR_ACCX                      "AccX"
#define ZSTR_ACCY                      "AccY"
#define ZSTR_ACCZ                      "AccZ"
#define ZSTR_GYROX                     "GYRX"
#define ZSTR_GYROY                     "GYRY"
#define ZSTR_GYROZ                     "GYRZ"
#define ZSTR_CURR                      "Curr"
#define ZSTR_CAPACITY                  "Capa"
#define ZSTR_VFAS                      "VFAS"
#define ZSTR_BATT_PERCENT              "Bat%"
#define ZSTR_ASPD                      "ASpd"
#define ZSTR_GSPD                      "GSpd"
#define ZSTR_HDG                       "Hdg"
#define ZSTR_SATELLITES                "Sats"
#define ZSTR_CELLS                     "Cels"
#define ZSTR_GPSALT                    "GAlt"
#define ZSTR_GPSDATETIME               "Date"
#define ZSTR_GPS                       "GPS"
#define ZSTR_BATT1_VOLTAGE             "RB1V"
#define ZSTR_BATT2_VOLTAGE             "RB2V"
#define ZSTR_BATT1_CURRENT             "RB1A"
#define ZSTR_BATT2_CURRENT             "RB2A"
#define ZSTR_BATT1_CONSUMPTION         "RB1C"
#define ZSTR_BATT2_CONSUMPTION         "RB2C"
#define ZSTR_BATT1_TEMP                "RB1T"
#define ZSTR_BATT2_TEMP                "RB2T"
#define ZSTR_RB_STATE                  "RBS"
#define ZSTR_CHANS_STATE               "RBCS"
#define ZSTR_RX_RSSI1                  "1RSS"
#define ZSTR_RX_RSSI2                  "2RSS"
#define ZSTR_RX_QUALITY                "RQly"
#define ZSTR_RX_SNR                    "RSNR"
#define ZSTR_RX_NOISE                  "RNse"
#define ZSTR_ANTENNA                   "ANT"
#define ZSTR_RF_MODE                   "RFMD"
#define ZSTR_TX_POWER                  "TPWR"
#define ZSTR_TX_RSSI                   "TRSS"
#define ZSTR_TX_QUALITY                "TQly"
#define ZSTR_TX_SNR                    "TSNR"
#define ZSTR_TX_NOISE                  "TNse"
#define ZSTR_PITCH                     "Ptch"
#define ZSTR_ROLL                      "Roll"
#define ZSTR_YAW                       "Yaw"
#define ZSTR_FLIGHT_MODE               "FM"
#define ZSTR_THROTTLE                  "Thr"
#define ZSTR_QOS_A                     "FdeA"
#define ZSTR_QOS_B                     "FdeB"
#define ZSTR_QOS_L                     "FdeL"
#define ZSTR_QOS_R                     "FdeR"
#define ZSTR_QOS_F                     "FLss"
#define ZSTR_QOS_H                     "Hold"
#define ZSTR_BIND                      "BIND"
#define ZSTR_LAP_NUMBER                "Lap "
#define ZSTR_GATE_NUMBER               "Gate"
#define ZSTR_LAP_TIME                  "LapT"
#define ZSTR_GATE_TIME                 "GteT"
#define ZSTR_ESC_VOLTAGE               "EscV"
#define ZSTR_ESC_CURRENT               "EscA"
#define ZSTR_ESC_RPM                   "Erpm"
#define ZSTR_ESC_CONSUMPTION           "EscC"
#define ZSTR_ESC_TEMP                  "EscT"
#define ZSTR_SD1_CHANNEL               "Chan"
#define ZSTR_GASSUIT_TEMP1             "GTp1"
#define ZSTR_GASSUIT_TEMP2             "GTp2"
#define ZSTR_GASSUIT_RPM               "GRPM"
#define ZSTR_GASSUIT_FLOW              "GFlo"
#define ZSTR_GASSUIT_CONS              "GFue"
#define ZSTR_GASSUIT_RES_VOL           "GRVl"
#define ZSTR_GASSUIT_RES_PERC          "GRPc"
#define ZSTR_GASSUIT_MAX_FLOW          "GMFl"
#define ZSTR_GASSUIT_AVG_FLOW          "GAFl"
#define ZSTR_SBEC_VOLTAGE              "BecV"
#define ZSTR_SBEC_CURRENT              "BecA"
#define ZSTR_RB3040_EXTRA_STATE        "RBES"
#define ZSTR_RB3040_CHANNEL1           "CH1A"
#define ZSTR_RB3040_CHANNEL2           "CH2A"
#define ZSTR_RB3040_CHANNEL3           "CH3A"
#define ZSTR_RB3040_CHANNEL4           "CH4A"
#define ZSTR_RB3040_CHANNEL5           "CH5A"
#define ZSTR_RB3040_CHANNEL6           "CH6A"
#define ZSTR_RB3040_CHANNEL7           "CH7A"
#define ZSTR_RB3040_CHANNEL8           "CH8A"
#define ZSTR_ESC_VIN                   "EVIN"
#define ZSTR_ESC_TFET                  "TFET"
#define ZSTR_ESC_CUR                   "ECUR"
#define ZSTR_ESC_TBEC                  "TBEC"
#define ZSTR_ESC_BCUR                  "CBEC"
#define ZSTR_ESC_VBEC                  "VBEC"
#define ZSTR_ESC_THR                   "ETHR"
#define ZSTR_ESC_POUT                  "EOUT"
#define ZSTR_SMART_BAT_BTMP            "BTmp"
#define ZSTR_SMART_BAT_BCUR            "BCur"
#define ZSTR_SMART_BAT_BCAP            "BUse"
#define ZSTR_SMART_BAT_MIN_CEL         "CLMi"
#define ZSTR_SMART_BAT_MAX_CEL         "CLMa"
#define ZSTR_SMART_BAT_CYCLES          "Cycl"
#define ZSTR_SMART_BAT_CAPACITY        "BCpT"
#define ZSTR_CL01                      "Cel1"
#define ZSTR_CL02                      "Cel2"
#define ZSTR_CL03                      "Cel3"
#define ZSTR_CL04                      "Cel4"
#define ZSTR_CL05                      "Cel5"
#define ZSTR_CL06                      "Cel6"
#define ZSTR_CL07                      "Cel7"
#define ZSTR_CL08                      "Cel8"
#define ZSTR_CL09                      "Cel9"
#define ZSTR_CL10                      "Cl10"
#define ZSTR_CL11                      "Cl11"
#define ZSTR_CL12                      "Cl12"
#define ZSTR_CL13                      "Cl13"
#define ZSTR_CL14                      "Cl14"
#define ZSTR_CL15                      "Cl15"
#define ZSTR_CL16                      "Cl16"
#define ZSTR_CL17                      "Cl17"
#define ZSTR_CL18                      "Cl18"
#define ZSTR_FRAME_RATE                "FRat"
#define ZSTR_TOTAL_LATENCY             "TLat"
#define ZSTR_VTX_FREQ                  "VFrq"
#define ZSTR_VTX_PWR                   "VPwr"
#define ZSTR_VTX_CHAN                  "VChn"
#define ZSTR_VTX_BAND                  "VBan"
#define ZSTR_SERVO_CURRENT             "SrvA"
#define ZSTR_SERVO_VOLTAGE             "SrvV"
#define ZSTR_SERVO_TEMPERATURE         "SrvT"
#define ZSTR_SERVO_STATUS              "SrvS"



struct CrossfireSensor {
  const uint8_t id;
  const uint8_t subId;
  const char * name;
  const TelemetryUnit unit;
  const uint8_t precision;
};


enum CrossfireSensorIndexes {
  RX_RSSI1_INDEX,
  RX_RSSI2_INDEX,
  RX_QUALITY_INDEX,
  RX_SNR_INDEX,
  RX_ANTENNA_INDEX,
  RF_MODE_INDEX,
  TX_POWER_INDEX,
  TX_RSSI_INDEX,
  TX_QUALITY_INDEX,
  TX_SNR_INDEX,
  BATT_VOLTAGE_INDEX,
  BATT_CURRENT_INDEX,
  BATT_CAPACITY_INDEX,
  BATT_REMAINING_INDEX,
  GPS_LATITUDE_INDEX,
  GPS_LONGITUDE_INDEX,
  GPS_GROUND_SPEED_INDEX,
  GPS_HEADING_INDEX,
  GPS_ALTITUDE_INDEX,
  GPS_SATELLITES_INDEX,
  ATTITUDE_PITCH_INDEX,
  ATTITUDE_ROLL_INDEX,
  ATTITUDE_YAW_INDEX,
  FLIGHT_MODE_INDEX,
  VERTICAL_SPEED_INDEX,
  UNKNOWN_INDEX,
};

// Frame id
#define GPS_ID                         0x02
#define CF_VARIO_ID                    0x07
#define BATTERY_ID                     0x08
#define LINK_ID                        0x14
#define CHANNELS_ID                    0x16
#define ATTITUDE_ID                    0x1E
#define FLIGHT_MODE_ID                 0x21
#define PING_DEVICES_ID                0x28
#define DEVICE_INFO_ID                 0x29
#define REQUEST_SETTINGS_ID            0x2A
#define COMMAND_ID                     0x32
#define RADIO_ID                       0x3A
