/**
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewara
 */

#include "BLEDevice.h"
//#include "BLEScan.h"


#define S_ID1 "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define S_ID2 "4fafc202-1fb5-459e-8fcc-c5c9c331914b"
#define S_ID3 "4fafc203-1fb5-459e-8fcc-c5c9c331914b"
#define S_ID4 "4fafc204-1fb5-459e-8fcc-c5c9c331914b"
#define C_ID1 "beb54831-36e1-4688-b7f5-ea07361b26a8"
#define C_ID2 "beb54832-36e1-4688-b7f5-ea07361b26a8"
#define C_ID3 "beb54833-36e1-4688-b7f5-ea07361b26a8"
#define C_ID4 "beb54834-36e1-4688-b7f5-ea07361b26a8"
// The remote service we wish to connect to.
BLEUUID serviceUUID[4];
// The characteristic of the remote service we are interested in.
BLEUUID charUUID[4];

typedef{
  BLEUUID serviceUUID;
  BLEUUID charUUID;
  uint8_t is_Connect;
  uint16_t Connect_id;
  BLERemoteCharacteristic *pRemoteCharacteristic;
}struct Client_t;

Client_t BLE_Client_struct[4];

void BLE_Client_Init(){
  for(i = 0; i < 4; i++)
    BLE_Client_struct[i].is_Connect = 0;
  BLE_Client_struct[0].serviceUUID.fromString((std::string)S_ID1);
  BLE_Client_struct[1].serviceUUID.fromString((std::string)S_ID2);
  BLE_Client_struct[2].serviceUUID.fromString((std::string)S_ID3);
  BLE_Client_struct[3].serviceUUID.fromString((std::string)S_ID4);
  BLE_Client_struct[0].charUUID.fromString((std::string)C_ID1);
  BLE_Client_struct[1].charUUID.fromString((std::string)C_ID2);
  BLE_Client_struct[2].charUUID.fromString((std::string)C_ID3);
  BLE_Client_struct[3].charUUID.fromString((std::string)C_ID4);
}

int i;

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic[4];
static BLEAdvertisedDevice* myDevice;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    for(i = 0; i < 4; i++){
      if(pclient -> getConnId() == BLE_Client_struct[i].Connect_id){
        Serial.print("Disconnected Client ");
        Serial.println(i);
        BLE_Client_struct[i].is_Connect = 0;
        break;
      }
    }
  }
};

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID()) {
      for(i = 0; i < 4; i++){
        if(BLE_Client_struct[i].is_Connect){
          continue;
        }
        if(advertisedDevice.isAdvertisingService(BLE_Client_struct[i].serviceUUID)){
          BLEDevice::getScan()->stop();
          myDevice = new BLEAdvertisedDevice(advertisedDevice);
          doConnect = true;
          doScan = true;
          break;
        }
      }
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    for(i = 0; i < 4; i++){
      if(BLE_Client_struct[i].is_Connect)
        continue;
      BLERemoteService* pRemoteService = pClient->getService(BLE_Client_struct[i].serviceUUID);
      if(pRemoteService != nullptr){
        break;
      }
    }
    if (pRemoteService == nullptr) {
      Serial.println("Failed to find our service");
      pClient->disconnect();
      return false;
    }
    Serial.print(" - Found our service ");
    Serial.println(i);


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    BLE_Client_struct[i].pRemoteCharacteristic = pRemoteService->getCharacteristic(BLE_Client_struct[i].charUUID);
    if (BLE_Client_struct[i].pRemoteCharacteristic == nullptr) {
      Serial.println("Failed to find our characteristic UUID");
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");
    Serial.print("ConnID = ");
    Serial.println(pClient->getConnId());

    // Read the value of the characteristic.

    if(BLE_Client_struct[i].pRemoteCharacteristic->canNotify())
      BLE_Client_struct[i].pRemoteCharacteristic->registerForNotify(notifyCallback);
    BLE_Client_struct[i].Connect_id = pClient->getConnId();
    BLE_Client_struct[i].is_Connect = 1;
    return true;
}



void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
} // End of setup.


// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    String TxValue;
    if(Serial.available()){
      TxValue = Serial.readString();
      pRemoteCharacteristic->writeValue(TxValue.c_str(), TxValue.length());
      Serial.print("Write on charUUID:");
      Serial.println(serviceUUID.toString().c_str());
      Serial.print("Send message:");
      Serial.println(TxValue);
    }
    //String newValue = "Time since boot: " + String(millis()/1000);
    //Serial.println("Setting new characteristic value to \"" + newValue + "\"");
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    //pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  delay(1000); // Delay a second between loops.
} // End of loop
