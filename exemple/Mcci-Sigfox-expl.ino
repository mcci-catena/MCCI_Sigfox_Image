//#include "MCCI_Sigfox_Image.h"

#include "MCCI_Sigfox.h"

// @TODO
// - Comprendre pourquoi l'init courte ne marche pas
// - Comprendre pourquoi ca a l'air de partir en sucette avec power > 14

/* ***************************************************************************
 * This is one of the first ways toi setup the library
 * You can define your own function. As an exemple if you need to
 * store the ids in the eeprom for an industrial production
 * or if you setup them using a serial line then in eeprom
 * you can rewrite these function ot make it working.
 * They are callback function the library use to acces the informations
 */
/*
uint8_t getCurrentRegion(uint32_t * region) {
  *region = REGION_RC2;
  return 0;
}

uint8_t getDeviceId(uint32_t * devId) {
  *devId = 0x14158D8;
  return 0;
}

uint8_t getInitialPac(uint8_t * pac) {
  static uint8_t _pac[8] = { 0x04,0x48,0x4D,0xF3,0xE2,0x26,0xF0,0x87 };
  bcopy(_pac,pac,8);
  return 0;
}

uint8_t getDeviceKey(uint8_t * key) {
  static uint8_t _key[16] = { 0x96,0xE0,0xAA,0xC5,0x65,0x13,0x9E,0x0C,0x9A,0x82,0x78,0xE0,0xDB,0x04,0x3A,0x8C };
  bcopy(_key,key,16);
  return 0;
}

uint8_t getTxPower(int8_t * power) {
  *power = 14; //DEFAULT_TXPOWER;
  return 0;
}

boolean serialReady = false;
void printLog( char * msg ) {
  if ( serialReady ) Serial.print(msg);
}

sigfox_api_t sigfoxApi = {
  getCurrentRegion,
  getDeviceId,
  getInitialPac,
  getDeviceKey,
  getTxPower,
  printLog,
  0x8080010
};
MCCI_Sigfox Sigfox(&sigfoxApi); 
*/

/* ***************************************************************************
 * This is a second way to setup the library.
 * Here we use string to make the configuration, this is the easiest way for
 * on prototype with a manual setup.
 * We pass the device ID, it needs to be 8 HexDigit, Pac and Key. The region for
 * the communication and at the end the EEPROM base address to be used.
 */

MCCI_Sigfox Sigfox( "014158D8", "04484DF3E226F087", "96E0AAC565139E0C9A8278E0DB043A8C", REGION_RC2, 0x8080010);  


/* ***************************************************************************
 * This is the last way to setup the library.
 * Compared to the previous one we are using binary data instead of strings
 * That way the memory search for credential is less easy even if this is not
 * a good way to protect credentials.
 * We pass the same element with different types.
 */
/*
/*
static uint8_t __pac[8] = { 0x04,0x48,0x4D,0xF3,0xE2,0x26,0xF0,0x87 };
static uint8_t __key[16] = { 0x96,0xE0,0xAA,0xC5,0x65,0x13,0x9E,0x0C,0x9A,0x82,0x78,0xE0,0xDB,0x04,0x3A,0x8C };
MCCI_Sigfox Sigfox( 0x014158D8, __pac, __key, REGION_RC2, 0x8080010);  
*/


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("GO !");
  //serialReady=true;
  for (int i=0 ; i < 3 ; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  // This function allows to ensure the library initialization has been
  // correctly executed.
  if ( Sigfox.isReady() ) {
    // When using the simple initilization, you can associate in a second step a logger
    // The library will print information only if the logs are enable in the config.h file.
    Sigfox.setLogger(&Serial);

    // Now let's print some of the information we can get from the library
    Serial.println("Sigfox library is ready");
    Serial.print(" RC : "); Serial.println(Sigfox.getCurrentRC());
    Serial.print(" DevId: 0x"); Serial.println(Sigfox.getDeviceId(),16);
    Serial.print(" InitialPac: ");
    uint8_t lpac[8];
    Sigfox.getInitialPac(lpac);
    for (int i = 0 ; i < 8 ; i++) {
      Serial.print(lpac[i],16);
    }
    Serial.println();
    Serial.print(" Last Reception Rssi: ");Serial.print(Sigfox.getLastRssi());Serial.println("dBm");
    Serial.print(" Last Sequence number used: ");Serial.println(Sigfox.getLastSeqId());
    Serial.print(" Sigfox version: ");Sigfox.printSigfoxVersion();
  } else {
    Serial.println("Failed to init Sigfox Library");
    while(true);
  }
}


void loop() {
  uint8_t mess[12] = { 0,1,2,3,4,5,6,7,8,9,10,11 }; 

  Serial.println("Fire a Sigfox 1 bit message");
  if ( Sigfox.sendBit(true) == MCCSIG_SUCCESS ) {
    Serial.println("... Success");
  } else {
    Serial.println("... Failed");
  }

  delay(1*60*1000);

  Serial.println("Fire a Sigfox message 8 Bytes");
  if ( Sigfox.sendFrame(mess,8) == MCCSIG_SUCCESS ) {
    Serial.println("... Success");
  } else {
    Serial.println("... Failed");
  }

  delay(19*60*1000);
} 