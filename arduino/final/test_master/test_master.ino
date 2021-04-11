#include <pcnt.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


//---------------------------------Slave Part-----------------------------


typedef struct {
  uint8_t Status;
  uint8_t Next_Status;
  uint8_t Data_Send;
} slave_t;

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


int i;
uint8_t Cnt_Inv, Have_Goal;
int16_t Position, Goal_Position;
uint8_t Motor_Status, Motor_Next_Status;

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
      }
      else if(Cnt_Inv && (Position <= Goal_Position)){
        motor_stop();
        Motor_Status = 0x01;
        Have_Goal = 0;
      }
    }
    else{
      if(Motor_Next_Status == 0x02){
        Goal_Position = mid_pos;
        if(Position < mid_pos){
          Motor_Status = 0x02;
          pcnt_set_direction(0);
          Have_Goal = 1;
          motor_forward();
        }
        else if(Position > mid_pos){
          Motor_Status = 0x02;
          pcnt_set_direction(1);
          Have_Goal = 1;
          motor_backward();
        }
        else{
          Motor_Status = 0x01;
        }
      }
      else if(Motor_Next_Status == 0x03){
        if(Position < max_pos){
          Motor_Status = 0x03;
          pcnt_set_direction(0);
          Goal_Position = max_pos;
          Have_Goal = 1;
          motor_forward();
        }
      }
      else if(Motor_Next_Status == 0x04){
        if(Position > min_pos){
          Motor_Status = 0x04;
          pcnt_set_direction(1);
          Goal_Position = min_pos;
          Have_Goal = 1;
          motor_backward();
        }
      }
      else if(Motor_Next_Status == 0x10){
        Motor_Status = 0x10;
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



//-------------------------------------BLE related--------------------------------------

#define SERVICE_UUID              "4fafc200-1fb5-459e-8fcc-c5c9c331914b"
static BLEUUID CHARACTERISTIC_UUID_HOST("beb54830-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID CHARACTERISTIC_UUID_1("beb54831-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID CHARACTERISTIC_UUID_2("beb54832-36e1-4688-b7f5-ea07361b26a8");
#define MAX_client 3

int j;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic_Host = NULL, *pCharacteristic_1 = NULL, *pCharacteristic_2 = NULL;
int client_num, old_client_num;
uint8_t BLE_data_get = 0;
BLEUUID new_data_UUID;
std::string RXvalue;
slave_t slave[5];
uint8_t send_data[10];
uint8_t send_data_length;
uint8_t master_status, master_next_status;
uint8_t Step, New_Data_TX;
uint8_t Go_mid;

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
    
    RXvalue = pCharacteristic->getValue();
    new_data_UUID = pCharacteristic->getUUID();
    BLE_data_get = 1;
  }
};

//BLE initializing
void BLE_init(){

  master_status = 0x00;
  master_next_status = 0x00;
  Step = 0;
  // Create the BLE Device
  BLEDevice::init("ESP1");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic_Host = pService->createCharacteristic(CHARACTERISTIC_UUID_HOST, 
                                                  BLECharacteristic::PROPERTY_NOTIFY|
                                                  BLECharacteristic::PROPERTY_WRITE
                                                  );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic_Host->addDescriptor(new BLE2902());
  pCharacteristic_Host->setCallbacks(new MyCallbacks());

  for(i = 0; i < 5; i++){
    slave[i].Status = 0x00;
    slave[i].Next_Status = 0x00;
  }

  pCharacteristic_1 = pService->createCharacteristic(CHARACTERISTIC_UUID_1,
                                                  BLECharacteristic::PROPERTY_NOTIFY|
                                                  BLECharacteristic::PROPERTY_WRITE
                                                  );
  pCharacteristic_2 = pService->createCharacteristic(CHARACTERISTIC_UUID_2,
                                                  BLECharacteristic::PROPERTY_NOTIFY|
                                                  BLECharacteristic::PROPERTY_WRITE
                                                  );
  pCharacteristic_1->addDescriptor(new BLE2902());
  pCharacteristic_1->setCallbacks(new MyCallbacks());
  pCharacteristic_2->addDescriptor(new BLE2902());
  pCharacteristic_2->setCallbacks(new MyCallbacks());

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

//Status Check
uint8_t status_check(uint8_t Status = 0x01){
  uint8_t All_Ready = 1;
  if(Motor_Status != Status)
    All_Ready = 0;
  for(i = 0; i < 5; i++){
    if(slave[i].Status != Status)
      All_Ready = 0;
  }
  Serial.print("Check for ");
  Serial.println(Status, HEX);
  return All_Ready;
}

//Step Set
void Step_1(){
  vTaskDelay(500);
  Motor_Next_Status = 0x03;
  slave[0].Next_Status = 0x03;
  slave[1].Next_Status = 0x03;
  slave[2].Next_Status = 0x04;
  slave[3].Next_Status = 0x04;
  slave[4].Next_Status = 0x04;
}

void Step_2(){
  vTaskDelay(500);
  Motor_Next_Status = 0x04;
  slave[0].Next_Status = 0x04;
  slave[1].Next_Status = 0x04;
  slave[2].Next_Status = 0x03;
  slave[3].Next_Status = 0x03;
  slave[4].Next_Status = 0x03;
}

void Step_mid(){
  vTaskDelay(500);
  Motor_Next_Status = 0x02;
  slave[0].Next_Status = 0x02;
  slave[1].Next_Status = 0x02;
  slave[2].Next_Status = 0x02;
  slave[3].Next_Status = 0x02;
  slave[4].Next_Status = 0x02;
  slave[5].Next_Status = 0x02;
}

void Step_Stop(){
  Motor_Next_Status = 0x01;
  slave[0].Next_Status = 0x01;
  slave[1].Next_Status = 0x01;
  slave[2].Next_Status = 0x01;
  slave[3].Next_Status = 0x01;
  slave[4].Next_Status = 0x01;
}

//Master Control Task
void master_control_task(void *parameter){
  (void) parameter;
  for(;;){
    if(master_next_status == 0x10){
      Serial.println("clear");
      master_status = 0x10;
      Motor_Next_Status = 0x10;
      Go_mid = 0;
      for(i = 0; i < 5; i++){
        slave[i].Next_Status = 0x10;
      }
      
      master_next_status = 0x01;
    }
    else if(!Go_mid && master_status == 0x10){
      if(status_check(0x11)){
        Step_mid();
        Go_mid = 1;
      }
      if(Go_mid && status_check()){
        Step_Stop();
        master_status = 0x01;
        master_next_status = 0x01;
      }
    }
    else if(master_status == 0x01){
      master_status = master_next_status;
      master_next_status = 0x01;
    }
    else if((master_status == 0x02) && status_check()){
      if(Step == 0){
        Step_1();
        Step = 1;
      }
      else if(Step == 1){
        Step_mid();
        Step = 2;
      }
      else if(Step == 2){
        master_status = 0x01;
        Step_Stop();
        Step = 3;
      }
      else if(Step == 3){
        Step_2();
        Step = 4;
      }
      else if(Step == 4){
        Step_mid();
        Step = 5;
      }
      else if(Step == 5){
        master_status = 0x01;
        Step_Stop();
        Step = 0;
      }
    }
    else if((master_status == 0x03) && status_check()){
      if(Step == 0){
        Step_2();
        Step = 5;
      }
      else if(Step == 5){
        Step_mid();
        Step = 4;
      }
      else if(Step == 4){
        master_status = 0x01;
        Step_Stop();
        Step = 3;
      }
      else if(Step == 3){
        Step_1();
        Step = 2;
      }
      else if(Step == 2){
        Step_mid();
        Step = 1;
      }
      else if(Step == 1){
        master_status = 0x01;
        Step_Stop();
        Step = 0;
      }
    }
    else if((master_status == 0x06) && status_check()){
      if(Step == 0){
        Step_1();
        Step = 1;
      }
      else if(Step == 1){
        Step_mid();
        Step = 2;
      }
      else if(Step == 2){
        if(master_next_status == 0x08){
          master_status = 0x01;
          Step_Stop();
        }
        Step = 3;
      }
      else if(Step == 3){
        Step_2();
        Step = 4;
      }
      else if(Step == 4){
        Step_mid();
        Step = 5;
      }
      else if(Step == 5){
        if(master_next_status == 0x08){
          master_status = 0x01;
          Step_Stop();
        }
        Step = 0;
      }
    }
    else if((master_status == 0x07) && status_check()){
      if(Step == 0){
        Step_2();
        Step = 5;
      }
      else if(Step == 5){
        Step_mid();
        Step = 4;
      }
      else if(Step == 4){
        if(master_next_status == 0x08){
          master_status = 0x01;
          Step_Stop();
        }
        Step = 3;
      }
      else if(Step == 3){
        Step_1();
        Step = 2;
      }
      else if(Step == 2){
        Step_mid();
        Step = 1;
      }
      else if(Step == 1){
        if(master_next_status == 0x08){
          master_status = 0x01;
          Step_Stop();
        }
        Step = 0;
      }
    }
    else{
      int All_Done = 1;
      if(Motor_Status == 0x01)
        All_Done = 0;
      for(i = 0; i < 5; i++){
        if(slave[i].Status == 0x01)
          All_Done = 0;
      }
      if(All_Done){
        Motor_Next_Status = 0x01;
        for(i = 0; i < 5; i++){
          slave[i].Next_Status = 0x01;
        }
      }
    }
    vTaskDelay(1000);
  }
}

//Data task
void data_task(void *parameter){
  (void) parameter;
  String TxValue;
  for(;;){
    if(BLE_data_get > 0){
      BLEUUID new_UUID = new_data_UUID;
      std::string RX_value = RXvalue;
      BLE_data_get = 0;
      Serial.print("From charUUID:");
      Serial.println(new_UUID.toString().c_str());
      Serial.print("Received data: ");
      for (int i = 0; i < RX_value.length(); i++)
        Serial.print(RX_value[i], HEX);
      Serial.println();
      if(new_UUID.equals(CHARACTERISTIC_UUID_HOST) && RX_value[0] == 0x0A){
        if(master_status == 0x00){
          New_Data_TX = 1;
          send_data[0] = 0x0A;
          send_data[1] = 0x00;
          send_data_length = 2;
        }
        else{
          master_next_status = RX_value[1];
          Serial.print("master_next_status = ");
          Serial.println(master_next_status, HEX);
        }
      }
      else if(new_UUID.equals(CHARACTERISTIC_UUID_1) && RX_value[0] == 0x0A){
        if(RX_value[1] < 2){
          (slave + RX_value[1]) -> Status = RX_value[2];
          Serial.print("Slave");
          Serial.print(RX_value[1], DEC);
          Serial.print(".Status = ");
          Serial.println(RX_value[2], HEX);
        }
        else{
          Serial.println("Returned Data ERROR");
        }
      }
      else if(new_UUID.equals(CHARACTERISTIC_UUID_2) && RX_value[0] == 0x0A){
        if(RX_value[1] < 5 && RX_value[1] > 1){
          (slave + RX_value[1]) -> Status = RX_value[2];
          Serial.print("Slave");
          Serial.print(RX_value[1] , DEC);
          Serial.print(".Status = ");
          Serial.println(RX_value[2], HEX);
        }
        else{
          Serial.println("Returned Data ERROR");
        }
      }
      vTaskDelay(100);
    }


    if(New_Data_TX){
      if (client_num > 0) {
        pCharacteristic_Host->setValue(send_data, send_data_length);
        pCharacteristic_Host->notify();
        Serial.print("Notify on charUUID:");
        Serial.println(pCharacteristic_Host->getUUID().toString().c_str());
        Serial.print("Notify message is:");
        for(i = 0; i < send_data_length; i++){
          Serial.print(send_data[i], HEX);
        }
        Serial.println();
        vTaskDelay(100); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
      }
      else{
        Serial.println("No device connected. So the message hasn't been send out");
      }
      New_Data_TX = 0;
    }
  

    //slave data part
    for(int i = 0; i < 5; i++){
      if(((slave[i].Status == 0x01) && (slave[i].Next_Status != 0x01)) || ((slave[i].Status == 0x11) && (slave[i].Next_Status == 0x02))){
        send_data[0] = 0x0A;
        send_data[1] = i;
        send_data[2] = slave[i].Next_Status;
        Serial.print("Notify on slave ");
        Serial.println(i);
        Serial.print("Next_Status = ");
        Serial.println(send_data[2], HEX);
        if(i < 2){
          pCharacteristic_1->setValue(send_data, 3);
          pCharacteristic_1->notify();
        }
        else{
          pCharacteristic_2->setValue(send_data, 3);
          pCharacteristic_2->notify();
        }
        vTaskDelay(100);
      }
    }

    // Client number changed(new device connects or old device disconnects)
    if(client_num != old_client_num){
      vTaskDelay(100);
      Serial.print(client_num);
      Serial.println(" devices connected");
      old_client_num = client_num;
      if(client_num < 3){
        vTaskDelay(1000); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
      }
    }
    int All_Done = 1;
    for(i = 0; i < 5; i++){
      if(slave[i].Status == 0x00)
        All_Done = 0;
    }
    if(client_num == MAX_client && master_status == 0x00 && All_Done){
      master_status = 0x01;
      master_next_status = 0x01;
    }
    if(client_num != MAX_client){
      master_status = 0x00;
      master_next_status = 0x00;
    }
    vTaskDelay(100);
  }
}

//Main Code
void setup() {
  Serial.begin(115200);
  pinMode(OE_pin, OUTPUT);
  digitalWrite(OE_pin, HIGH);
  motor_init();
  cnt_init();
  BLE_init();
  for(i = 0; i < 5; i++){
    slave[i].Next_Status = 0x01;
  }
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
  

  xTaskCreatePinnedToCore(
    master_control_task
    ,  "master_control_task"
    ,  10000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  0);


    
}

void loop() {

}
