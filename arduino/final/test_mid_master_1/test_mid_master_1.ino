#include <pcnt.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//---------------------------------------------Motor and PCNT Part------------------------------

#define pin_1 13
#define pin_2 12
#define OE_pin 23
#define A_pin 32
#define B_pin 33
#define PWMchannel 0
#define freq 1000
#define reso 10
#define ideal_mid_pos 1332

uint16_t mid_pos = 1332;
uint16_t min_pos = 600;
uint16_t max_pos = 2064;


int i;
uint8_t Cnt_Inv, Have_Goal;
int16_t Position, Goal_Position;
uint8_t Motor_Status, Motor_Next_Status;
uint8_t BLE_data_send;

// Motor Part
void motor_init(){
  ledcSetup(0, freq, 10);
  ledcSetup(1, freq, 10);
  ledcAttachPin(pin_1, 0);
  ledcAttachPin(pin_2, 1);
}

void motor_forward(){
  ledcWrite(0, 0);
  ledcWrite(1, 512);
}

void motor_backward(){
  ledcWrite(0, 512);
  ledcWrite(1, 0);
}

void motor_stop(){
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}


//Pulse Counter Part
void cnt_init(){
  pcnt_config_t pcnt_config;
  pcnt_config.pulse_gpio_num = A_pin;
  pcnt_config.ctrl_gpio_num = -1;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = 3000;
  pcnt_config.counter_l_lim = -3000;
  Cnt_Inv = 0;
  pcnt_unit_config(&pcnt_config);
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  Have_Goal = 0;
  Motor_Status = 0x01;
}

//Pulse Counter Direction
void pcnt_set_direction(uint8_t dir){
  if(dir && !Cnt_Inv){
    pcnt_set_mode(PCNT_UNIT_0, PCNT_CHANNEL_0, PCNT_COUNT_DEC, PCNT_COUNT_DEC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
    Cnt_Inv = 1;
  }
  else if(!dir && Cnt_Inv){
    pcnt_set_mode(PCNT_UNIT_0, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
    Cnt_Inv = 0;
  }
}

//Pulse Counter and Motor Task
void motor_pcnt_task(void *parameter){
  (void) parameter;
  for(;;){
    pcnt_get_counter_value(PCNT_UNIT_0, &Position);
    if(Have_Goal){
      if(!Cnt_Inv && (Position >= Goal_Position)){
        motor_stop();
        Motor_Status = 0x01;
        Have_Goal = 0;
        Motor_Next_Status = 0x01;
        BLE_data_send = 1;
      }
      else if(Cnt_Inv && (Position <= Goal_Position)){
        motor_stop();
        Motor_Status = 0x01;
        Have_Goal = 0;
        Motor_Next_Status = 0x01;
        BLE_data_send = 1;
      }
    }
    else{
      if(Motor_Next_Status == 0x02){
        Goal_Position = mid_pos;
        if(Position < mid_pos - 10){
          Motor_Status = 0x02;
          pcnt_set_direction(0);
          Have_Goal = 1;
          motor_forward();
          BLE_data_send = 1;
        }
        else if(Position > mid_pos + 10){
          Motor_Status = 0x02;
          pcnt_set_direction(1);
          Have_Goal = 1;
          motor_backward();
          BLE_data_send = 1;
        }
        else{
          Motor_Status = 0x02;
          BLE_data_send = 1;
          vTaskDelay(2000);
          Motor_Status = 0x01;
          BLE_data_send = 1;
        }
      }
      else if(Motor_Next_Status == 0x03){
        if(Position < max_pos){
          Motor_Status = 0x03;
          pcnt_set_direction(0);
          Goal_Position = max_pos;
          Have_Goal = 1;
          motor_forward();
          BLE_data_send = 1;
        }
      }
      else if(Motor_Next_Status == 0x04){
        if(Position > min_pos){
          Motor_Status = 0x04;
          pcnt_set_direction(1);
          Goal_Position = min_pos;
          Have_Goal = 1;
          motor_backward();
          BLE_data_send = 1;
        }
      }
      else if(Motor_Next_Status == 0x10){
        Motor_Status = 0x10;
        BLE_data_send = 1;
        int Old_Position = Position;
        motor_backward();
        for(;;){
          vTaskDelay(2000);
          pcnt_get_counter_value(PCNT_UNIT_0, &Position);
          if(Old_Position == Position){
            pcnt_counter_pause(PCNT_UNIT_0);
            pcnt_counter_clear(PCNT_UNIT_0);
            pcnt_counter_resume(PCNT_UNIT_0);
            motor_stop();
            Motor_Status = 0x11;
            BLE_data_send = 1;
            break;
          }
          Old_Position = Position;
        }
      }
      Motor_Next_Status = 0x01;
    }
    vTaskDelay(10);
  }
}


//---------------------------------------------BLE Part----------------------------------------

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc200-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("beb54832-36e1-4688-b7f5-ea07361b26a8");

//Server private
#define SERVICE_UUID        "4fafc202-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID1 "beb54831-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID2 "beb54832-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic *pCharacteristic1 = NULL, *pCharacteristic2 = NULL;
int client_num, old_client_num;
uint32_t value = 0;
#define MAX_CLIENT 3

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
BLEUUID new_data_write_UUID, new_data_notify_UUID;
std::string write_data, notify_data;
uint8_t new_write_data = 0, new_notify_data = 0;
uint8_t TX_master_data[10], TX_slave_data[10];

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    client_num++;
  };
  void onDisconnect(BLEServer* pServer) {
    client_num--;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    write_data = pCharacteristic->getValue();
    if (write_data.length() > 0) {
      new_data_write_UUID = pCharacteristic -> getUUID();
      new_write_data = 1;
    }
  }
};

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    new_data_notify_UUID = pBLERemoteCharacteristic->getUUID();
    notify_data = (char*)pData;
    new_notify_data = 1;
    
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

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
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}
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
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void BLE_service_Init(){
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
    // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic1 = pService->createCharacteristic(CHARACTERISTIC_UUID1, 
                                                  BLECharacteristic::PROPERTY_NOTIFY|
                                                  BLECharacteristic::PROPERTY_WRITE
                                                  );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic1->addDescriptor(new BLE2902());
  pCharacteristic1->setCallbacks(new MyCallbacks());

  pCharacteristic2 = pService->createCharacteristic(CHARACTERISTIC_UUID2,
                                                  BLECharacteristic::PROPERTY_NOTIFY|
                                                  BLECharacteristic::PROPERTY_WRITE
                                                  );

  pCharacteristic2->addDescriptor(new BLE2902());
  pCharacteristic2->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void BLE_client_Init(){
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  i = 1;
  while(!doConnect){
    pBLEScan->start(5, false);
    Serial.print(i++);
    Serial.println(" time(s) scan starts");
    delay(6000);
    if(i == 10){
      Serial.println("Scan ERROR");
      break;
    }
  }
}

void data_task(void *parameter){
  (void) parameter;
  for(;;){
    vTaskDelay(100);
    if(doConnect == true){
      if(connectToServer()){
        Serial.println("We are now connected to the BLE Server.");
        vTaskDelay(1000);
        TX_master_data[0] = 0x0A;
        TX_master_data[1] = 0x02;
        TX_master_data[2] = 0x01;
        pRemoteCharacteristic -> writeValue(TX_master_data, 3);
        Motor_Status = 0x01;
      } else {
        Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      }
      doConnect = false;
    }

    if(connected){
      if(new_notify_data){
        Serial.print("Notify callback for characteristic ");
        Serial.print(new_data_notify_UUID.toString().c_str());
        Serial.print("data: ");
        for(i = 0; i < notify_data.length(); i++)
          Serial.print(notify_data[i], HEX);
        Serial.println();
        if(notify_data[0] == 0x0A){
          if(notify_data[1] == 0x02){
            if(Motor_Next_Status == notify_data[2]){
              BLE_data_send = 1;
            }
            else{
              Motor_Next_Status = notify_data[2];
            }
          }
          else if(notify_data[1] == 0x03 || notify_data[1] == 0x04){
            TX_slave_data[0] = 0x0A;
            TX_slave_data[1] = notify_data[2];
            BLECharacteristic *pCharacteristic;
            if(notify_data[1] == 0x03){
              pCharacteristic = pCharacteristic1;
            }
            else{
              pCharacteristic = pCharacteristic2;
            }
            pCharacteristic -> setValue(TX_slave_data, 2);
            pCharacteristic -> notify();
            Serial.print("Notify on charUUID:");
            Serial.println(pCharacteristic->getUUID().toString().c_str());
            Serial.print("Notify message is:");
            for(i = 0; i < 2; i++){
              Serial.print(TX_slave_data[i], HEX);
            }
            Serial.println();
            vTaskDelay(100);
          }
        }
        new_notify_data = 0;
      }

      if(new_write_data){
        Serial.print("Received write data on UUID:");
        Serial.println(new_data_write_UUID.toString().c_str());
        Serial.print("Received write message is:");
        for(i = 0; i < write_data.length(); i++){
          Serial.print(write_data[i], HEX);
        }
        Serial.println();
        if(write_data[0] == 0x0A){
          TX_master_data[0] = 0x0A;
          if(new_data_write_UUID.equals(BLEUUID(CHARACTERISTIC_UUID1)))
            TX_master_data[1] = 0x03;
          else
            TX_master_data[1] = 0x04;
          TX_master_data[2] = write_data[1];
          pRemoteCharacteristic -> writeValue(TX_master_data, 3);
          Serial.print("Slave data have been sent to the master. Data is:");
          for(i = 0; i < 3; i++){
            Serial.print(TX_master_data[i], HEX);
          }
          Serial.println();
        }
        new_write_data = 0;
        vTaskDelay(100);
      }

      if(BLE_data_send){
        TX_master_data[0] = 0x0A;
        TX_master_data[1] = 0x02;
        TX_master_data[2] = Motor_Status;
        pRemoteCharacteristic -> writeValue(TX_master_data, 3);
        Serial.print("Mid data have been sent to the master. Data is:");
        for(i = 0; i < 3; i++){
          Serial.print(TX_master_data[i], HEX);
        }
        Serial.println();
        BLE_data_send = 0;
        vTaskDelay(100);
      }  
    }


    //Server parameter

    if(client_num != old_client_num){
      delay(100);
      Serial.print(client_num);
      Serial.println(" devices connected");
      old_client_num = client_num;
      if(client_num < 3 && connected){
        delay(100); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
      }
    }

    if(doScan && !connected){
      BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  pinMode(OE_pin, OUTPUT);
  digitalWrite(OE_pin, HIGH);
  motor_init();
  cnt_init();
  BLEDevice::init("ESP3");
  BLE_service_Init();
  BLE_client_Init();
  
  //Create FreeRTOS Tasks
  xTaskCreatePinnedToCore(
    motor_pcnt_task
    ,  "motor_pcnt_task"
    ,  10000  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  1);


  xTaskCreatePinnedToCore(
    data_task
    ,  "data_task"
    ,  10000  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  0);
  

} // End of setup.


// This is the Arduino main loop function.
void loop() {

} // End of loop
