


uint8_t createCrossfireChannelsFrame(uint8_t * frame);
void startCrossfire();
void runCrossfire();
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
#define CROSSFIRE_SERIAL Serial3

#define EXBUS_LOW   8000
#define EXBUS_HIGH  16000

//high speed
#define REFRESH_INTERVAL 4  
#define CROSSFIRE_BAUD_RATE 400000 //baud

//slow speed
//#define REFRESH_INTERVAL 16  
//#define CROSSFIRE_BAUD_RATE 115200 //baud
