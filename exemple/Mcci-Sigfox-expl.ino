#include "MCCI_Sigfox_Image.h"

uint8_t getCurrentRegion(uint32_t * region) {
  *region = __LORAWAN_REGION_EU868;
  return 0;
}

uint8_t getDeviceId(uint32_t * devId) {
  *devId = 0x00000000;
  return 0;
}

uint8_t getInitialPac(uint8_t * pac) {
  static uint8_t _pac[8] = { 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00 };
  bcopy(_pac,pac,8);
  return 0;
}

uint8_t getInitialKey(uint8_t * key) {
  static uint8_t _key[16] = { 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00 };
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
