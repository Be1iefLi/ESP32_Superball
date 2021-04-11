#include <pcnt.h>
#include <BLEDevice.h>

//---------------------------------Motor and PCNT related-----------------

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


String data, TxData;
uint8_t Cnt_Inv, Have_Goal;
int16_t Position, Goal_Position;
uint8_t RX_status, TX_return;
uint8_t BLE_data_get = 0, BLE_data_send = 0;

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
        TX_return = 0x01;
        BLE_data_send = 1;
        Have_Goal = 0;
        RX_status = 0x01;
      }
      else if(Cnt_Inv && (Position <= Goal_Position)){
        motor_stop();
        TX_return = 0x01;
        BLE_data_send = 1;
        Have_Goal = 0;
        RX_status = 0x01;
      }
    }
    else if(BLE_data_get){
      BLE_data_get = 0;
      if(RX_status == 0x02){
        Goal_Position = mid_pos;
        if(Position < mid_pos - 10){
          pcnt_set_direction(0);
          Have_Goal = 1;
          motor_forward();
          TX_return = 0x02;
          BLE_data_send = 1;
        }
        else if(Position > mid_pos + 10){
          pcnt_set_direction(1);
          Have_Goal = 1;
          motor_backward();
          TX_return = 0x02;
          BLE_data_send = 1;
        }
        else{
          TX_return = 0x02;
          BLE_data_send = 1;
          vTaskDelay(2000);
          TX_return = 0x01;
          BLE_data_send = 1;
        }
      }
      else if(RX_status == 0x03){
        if(Position < max_pos){
          pcnt_set_direction(0);
          Goal_Position = max_pos;
          Have_Goal = 1;
          motor_forward();
          TX_return = 0x03;
          BLE_data_send = 1;
        }
      }
      else if(RX_status == 0x04){
        if(Position > min_pos){
          pcnt_set_direction(1);
          Goal_Position = min_pos;
          Have_Goal = 1;
          motor_backward();
          TX_return = 0x04;
          BLE_data_send = 1;
        }
      }
      else if(RX_status == 0x10){
        int Old_Position = Position;
        motor_backward();
        TX_return = 0x10;
        BLE_data_send = 1;
        for(;;){
          vTaskDelay(2000);
          pcnt_get_counter_value(PCNT_UNIT_0, &Position);
          if(Old_Position == Position){
            pcnt_counter_pause(PCNT_UNIT_0);
            pcnt_counter_clear(PCNT_UNIT_0);
            pcnt_counter_resume(PCNT_UNIT_0);
            motor_stop();
            TX_return = 0x11;
            BLE_data_send = 1;
            break;
          }
          Old_Position = Position;
        }
      }
    }
    vTaskDelay(10);
  }
}


//-------------------------------------BLE related--------------------------------------

#define serviceUUID    "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define charUUID       "beb54831-36e1-4688-b7f5-ea07361b26a8"

#define MAX_client 1

BLEServer* pServer = NULL;
uint8_t doConnect = false;
uint8_t connected = false;
uint8_t doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
uint8_t data_TX[10];
int i;


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
    for(int i = 0; i < length; i++){
      Serial.print(pData[i], HEX);
    }
    Serial.println();
    if(pData[0] = 0x0A){
      if(RX_status == pData[1])
        BLE_data_send = 1;
      else{
        RX_status = pData[1];
        BLE_data_get = 1;
      }
    }
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
      Serial.println(serviceUUID);
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID);
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canNotify()){
      pRemoteCharacteristic->registerForNotify(notifyCallback);
    }
    connected = true;
    return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(serviceUUID))) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

//BLE initializing
void BLE_init(){
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("ESP_Slave");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  int i = 1;
  while(!doConnect){
    if(i > 10){
      Serial.println("Scan ERROR");
      break;
    }
    pBLEScan->start(5, false);
    Serial.print(i++);
    Serial.println(" time(s) scan");
    delay(6000);
  }
  BLE_data_get = 0;
  BLE_data_send = 0;
}

//Data task
void data_task(void *parameter){
  (void) parameter;
  for(;;){
    if (doConnect == true) {
      if (connectToServer()) {
        Serial.println("We are now connected to the BLE Server.");
        vTaskDelay(1000);
        data_TX[0] = 0x0A;
        data_TX[1] = 0x01;
        pRemoteCharacteristic -> writeValue(data_TX, 2);
        TX_return = 0x01;
      } else {
        Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      }
      doConnect = false;
    }

    // If we are connected to a peer BLE Server, update the characteristic each time we are reached
    // with the current time since boot.
    if (connected) {
      if(BLE_data_send){
        data_TX[0] = 0x0A;
        data_TX[1] = TX_return;
        pRemoteCharacteristic -> writeValue(data_TX, 2);
        BLE_data_send = 0;
        Serial.print("Slave data have been sent to the mid. Data is:");
        for(i = 0; i < 2; i++){
          Serial.print(data_TX[i]);
        }
      }
      String TxValue;
      if(Serial.available()){
        TxValue = Serial.readString();
        pRemoteCharacteristic->writeValue(TxValue.c_str(), TxValue.length());
        Serial.print("Write on charUUID:");
        Serial.println(serviceUUID);
        Serial.print("Send message:");
        Serial.println(TxValue);
      }
      //String newValue = "Time since boot: " + String(millis()/1000);
      //Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      
      // Set the characteristic's value to be the array of bytes that is actually a string.
      //pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
    }
    else if(doScan){
        BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
    }
    
    vTaskDelay(100);
  }
}

//Main Code
void setup() {
  Serial.begin(115200);
  TX_return = 0x01;
  pinMode(OE_pin, OUTPUT);
  digitalWrite(OE_pin, HIGH);
  motor_init();
  cnt_init();
  BLE_init();
  //Create FreeRTOS Tasks
  xTaskCreatePinnedToCore(
    motor_pcnt_task
    ,  "motor_pcnt_task"   // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
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
}

void loop() {

}
