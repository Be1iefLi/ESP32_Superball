/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
BLECharacteristic * pRxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue[100];
uint8_t *tx_point;
char uart2_buf[100];
char output_buf[100];
bool sign;
std::string rxValue;
bool rx_signal = 0, BLE_output = 0;
int tx_length;
int uart2_buf_length;
int16_t acc[3], YPR[3];
char acc_out[20],ypr_out[20];

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  
    void onWrite(BLECharacteristic *pCharacteristic) {
      rx_signal = true;
      
    }
};

void gyro_rx(){
  int counter = 0;
  while(Serial2.available()){
    uart2_buf[counter] = Serial2.read();
    if(counter == 0 && uart2_buf[0] != 0x5a)
      continue;
    counter++;
    if(counter == 17)
      sign = 1;
  }
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  pinMode(19,OUTPUT);
  
  Serial2.write(0XA5); 
  Serial2.write(0X55);    
  Serial2.write(0X11);    //初始化GY25Z,输出加速度和欧拉角
  Serial2.write(0X0B); 
  delay(100); 
 
  Serial2.write(0XA5); 
  Serial2.write(0X56);    //初始化GY25Z,连续输出模式
  Serial2.write(0X02);    //初始化GY25Z,连续输出模式
  Serial2.write(0XFD);
  delay(100);  
}

void loop() {
    bool Is_send_data = false;

    if(Serial2.available()){//Get Gyroscope data
      gyro_rx();
      if(sign){
        sign = 0;
        if(uart2_buf[0] == 0x5a && uart2_buf[1] == 0x5a){
          acc[0] = (uart2_buf[4]<<8|uart2_buf[5]);//Get gyro data
          acc[1] = (uart2_buf[6]<<8|uart2_buf[7]);
          acc[2] = (uart2_buf[8]<<8|uart2_buf[9]);

          YPR[0] = (uart2_buf[10]<<8|uart2_buf[11]);
          YPR[1] = (uart2_buf[12]<<8|uart2_buf[13]);
          YPR[2] = (uart2_buf[14]<<8|uart2_buf[15]);

          sprintf(acc_out,"a:%i;%i;%i;",acc[0],acc[1],acc[2]);
          sprintf(ypr_out,"y:%i;%i;%i;",YPR[0],YPR[1],YPR[2]);
          //Serial.println(acc_out);
          //Serial.println(ypr_out);
          /*
          Serial.print("acc:\t");
          Serial.print(acc_out[0]); Serial.print("\t"); 
          Serial.print(acc_out[1]); Serial.print("\t"); 
          Serial.print(acc_out[2]);         
          Serial.print("  ,YPR:\t");
          Serial.print(YPR_out[2]); Serial.print("\t"); //显示航向
          Serial.print(YPR_out[1]); Serial.print("\t"); //显示俯仰角
          Serial.println(YPR_out[0]);                    //显示横滚角
          */  
        }
      }
    }

    if (rx_signal){//BLE rx
      rx_signal = false;
      if (deviceConnected){
        rxValue = pRxCharacteristic->getValue();

        if (rxValue.length() > 0) {
          Serial.println("*********");
          Serial.print("Received Value: ");
          for (int i = 0; i < rxValue.length(); i++)
            Serial.print(rxValue[i]);

          Serial.println();
          Serial.println("*********");
        }

        if(rxValue == "send")
          Is_send_data = true;
      }
    }

    if(Is_send_data){
      if (deviceConnected) {
        strcpy((char*)txValue,acc_out);
        tx_length = strlen(acc_out);
        pTxCharacteristic->setValue(txValue, tx_length);
        pTxCharacteristic->notify();
        strcpy((char*)txValue,ypr_out);
        delay(10);
        tx_length = strlen(ypr_out);
        pTxCharacteristic->setValue(txValue, tx_length);
        pTxCharacteristic->notify();
        Serial.println("Message has transmitted!");
      }
      delay(10);
    }

    /*
    if(Serial.available()){
      for(tx_length = 0; Serial.available(); tx_length++)
        txValue[tx_length] = Serial.read(); 
      txValue[tx_length] = 0;
      if (deviceConnected) {
        pTxCharacteristic->setValue(txValue, tx_length);
        pTxCharacteristic->notify();
        Serial.println("Message has transmitted!");
      }
		delay(10); // bluetooth stack will go into congestion, if too many packets are sent
	  }*/

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }

}
