#include "pcnt.h"

#define pin_1 13
#define pin_2 12
#define A_pin 32
#define B_pin 23
#define PWMchannel 0
#define freq 1000
#define reso 10
#define ideal_mid_pos 1332

uint16_t mid_pos = 1332;
uint16_t min_pos = 600;
uint16_t max_pos = 2064;

String data;
int Serial_data[20];
uint8_t Cnt_Inv, Have_Goal;
int16_t Position, Goal_Position;
uint8_t New_Data;

// Motor Part
void motor_init(){
  ledcSetup(0, freq, 10);
  ledcSetup(1, freq, 10);
  ledcAttachPin(pin_1, 0);
  ledcAttachPin(pin_2, 1);
}

void motor_forward(){
  ledcWrite(0, 0);
  ledcWrite(1, 700);
}

void motor_backward(){
  ledcWrite(0, 700);
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
        Serial.println("arrive");
        Have_Goal = 0;
      }
      else if(Cnt_Inv && (Position <= Goal_Position)){
        motor_stop();
        Serial.println("arrive");
        Have_Goal = 0;
      }
    }
    else if(New_Data){
      New_Data = 0;
      if(data.startsWith("stop")){
        Serial.println("Stop");
        motor_stop();
      }
      else if(data.startsWith("advance")){
        Serial.println("Forward");
        pcnt_set_direction(0);
        motor_forward();    
      }
      else if(data.startsWith("back")){
        Serial.println("Backward");
        pcnt_set_direction(1);
        motor_backward();
      }
      else if(data.startsWith("mid")){
        Goal_Position = mid_pos;
        if(Position < mid_pos){
          pcnt_set_direction(0);
          Have_Goal = 1;
          motor_forward();
        }
        else if(Position > mid_pos){
          pcnt_set_direction(1);
          Have_Goal = 1;
          motor_backward();
        }
      }
      else if(data.startsWith("max")){
        if(Position < max_pos){
          pcnt_set_direction(0);
          Goal_Position = max_pos;
          Have_Goal = 1;
          motor_forward();
        }
      }
      else if(data.startsWith("min")){
        if(Position > min_pos){
          pcnt_set_direction(1);
          Goal_Position = min_pos;
          Have_Goal = 1;
          motor_backward();
        }
      }
      else if(data.startsWith("clear")){
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
            Serial.println("Cleared");
            break;
          }
          Old_Position = Position;
        }
      }
      else if(data.startsWith("p")){
        Serial.print("Position = ");
        Serial.println(Position);
      }
    }
    vTaskDelay(10);
  }
}

//Data task
void data_task(void *parameter){
  (void) parameter;
  for(;;){
    if(Serial.available()){
      data = Serial.readString();
      Serial.print(data);
      New_Data = 1;
    }
    vTaskDelay(100);
  }
}

//Main Code
void setup() {
  New_Data = 0;
  Serial.begin(115200);
  motor_init();
  cnt_init();
  pinMode(B_pin, OUTPUT);
  digitalWrite(B_pin, HIGH);
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
