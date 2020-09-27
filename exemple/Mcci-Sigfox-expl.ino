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
  static uint8_t _pac[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
  bcopy(_pac,pac,8);
  return 0;
}

uint8_t getDeviceKey(uint8_t * key) {
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

void printLog( char * msg ) {
  Serial.print(msg);
}


sigfox_api_t sigfoxApi = {
  getCurrentRegion,
  getDeviceId,
  getInitialPac,
  getDeviceKey,
  getTxPower,
  printLog
};

void inter(void) {
  
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("GO !");
  for (int i=0 ; i < 3 ; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  sigfox_setup(&sigfoxApi);
}


void loop() {
  uint8_t mess[12] = { 0,1,2,3,4,5,6,7,8,9,10,11 }; 

  itsdk_sigfox_sendFrame(
          mess,             // Buffer
          12,               // Size
          2,                // repeat
          SIGFOX_SPEED_DEFAULT,
          SIGFOX_POWER_DEFAULT,
          PAYLOAD_ENCRYPT_NONE,
          false,
          NULL
      );

   delay(10*60*1000);

} 
