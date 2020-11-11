#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <EEPROM.h>
#include "esp_gap_bt_api.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define LOOP_DELAY 1000
#define EEPROM_SIZE 4

uint64_t lastDebounceTime = 0;
unsigned long debounceDelay = 60000;//minut

BLECharacteristic *pCharacteristic;

unsigned int simpleCounter = 1;

void writeIntIntoEEPROM(int address, int number) { 
  EEPROM.write(address, number >> 24);
  EEPROM.write(address + 1, number >> 16);
  EEPROM.write(address + 2, number >> 8);
  EEPROM.write(address + 3, number & 0xFF);
  EEPROM.commit();
}

int readIntFromEEPROM(int address) {
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  byte byte3 = EEPROM.read(address + 2);
  byte byte4 = EEPROM.read(address + 3);
  return (byte1 << 24) + (byte2 << 16) + (byte3 << 8) + byte4;
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Device connected");
    BLEDevice::startAdvertising();
  };

  void onDisconnect(BLEServer* pServer) {
    Serial.println("Device disconnected");
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  
  simpleCounter = readIntFromEEPROM(0);
  
  BLEDevice::init("ESP");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
                                       
  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);//ako nije uparen nece moci da procita
  pCharacteristic->setValue(simpleCounter);
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  BLESecurity *pSecurity = new BLESecurity();
  uint32_t passkey = 123456;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));//postavljanje statickog pina jer void BLESecurity::setStaticPIN(uint32_t pin) ne radi
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
  pSecurity->setCapability(ESP_IO_CAP_OUT);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
}

void loop() {
  simpleCounter++;

  if((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    writeIntIntoEEPROM(0, simpleCounter);
  }
  
  String s = String(simpleCounter);
  pCharacteristic->setValue(s.c_str());
  pCharacteristic->notify(true);
  
  delay(LOOP_DELAY);
}
