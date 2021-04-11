#include <mcpwm.h>
#define PWM_A_pin 12
#define PWM_B_pin 13


String data;

void motor_init(){
  Serial.println("Initializing mcpwm gpio...");
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_A_pin);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_B_pin);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 2000;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void motor_forward(){
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void motor_backward(){
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 50);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void motor_stop(){
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B); 
}



void setup() {
  motor_init();
  Serial.begin(115200);
  Serial.println("Initialized");
}

void loop() {
  if(Serial.available()){
    data = Serial.readString();
    if(data.startsWith("a")){
      motor_stop();
      Serial.println("Stop");
    }
    else if(data.startsWith("b")){
      motor_forward();
      Serial.println("Forward");
    }
    else if(data.startsWith("c")){
      motor_backward();
      Serial.println("Backward");
    }
  }
  delay(10);
}
