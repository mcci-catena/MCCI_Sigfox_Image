#include "MCCI_Sigfox_Image.h"

uint8_t getCurrentRegion(uint32_t * region) {
  *region = __LORAWAN_REGION_EU868;
  return 0;
}

uint8_t getDeviceId(uint32_t * devId) {
  *devId = 0x00D206BF;
  return 0;
}

uint8_t getInitialPac(uint8_t * pac) {
  static uint8_t _pac[8] = { 0xEC, 0x3E, 0x61, 0xEB,
                              0xDA, 0xF7, 0xE6, 0x3C };
  bcopy(_pac,pac,8);
  return 0;
}

uint8_t getInitialKey(uint8_t * key) {
  static uint8_t _key[16] = { 0x74, 0xE6, 0x77, 0xC6,
                              0xE8, 0xEE, 0x7B, 0xC5,
                              0x65, 0xC0, 0xA5, 0x4E,
                              0x7E, 0xC8, 0xEA, 0xE9 };
  bcopy(_key,key,16);
  return 0;
}

uint8_t getTxPower(int8_t * power) {
  *power = 14;
  return 0;
}


sigfox_api_t sigfoxApi = {
  getCurrentRegion,
  getDeviceId,
  getInitialPac,
  getInitialKey,
  getTxPower
};

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  sigfox_setup(&sigfoxApi);
}



void loop() {
  uint8_t mess[4] = { 0,1,2,3 }; 

  itsdk_sigfox_sendFrame(
          mess,             // Buffer
          4,                // Size
          2,                // repeat
          SIGFOX_SPEED_DEFAULT,
          SIGFOX_POWER_DEFAULT,
          PAYLOAD_ENCRYPT_NONE,
          false,
          NULL
      );

} 
