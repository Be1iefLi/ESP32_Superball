/* MCPWM basic config example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use each submodule of MCPWM unit.
 * The example can't be used without modifying the code first.
 * Edit the macros at the top of mcpwm_example_basic_config.c to enable/disable the submodules which are used in the example.
 */

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "driver/mcpwm.h"
#include "driver/uart.h"
#include "soc/mcpwm_periph.h"

#define PWM_A_pin 13
#define PWM_B_pin 12
#define PCNT_A_pin 32
#define PCNT_B_pin 23

#define Mid_Pos 1332
#define Max_Pos 666

#define MOTOR_TASK_TAG "MOTOR_TASK"

void motor_forward();
void motor_backward();
void motor_stop();

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

static void IRAM_ATTR pcnt_intr_handler(void *arg)
{
    motor_stop();
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

static void motor_init(void)
{
    ESP_LOGI(MOTOR_TASK_TAG, "Initializing mcpwm gpio...");
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

static void pcnt_init(void)
{
    ESP_LOGI(MOTOR_TASK_TAG, "Initializing pulse counter...");
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_A_pin,
        .ctrl_gpio_num = -1,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = 32767,
        .counter_l_lim = -32767,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_1, Max_Pos);
    //pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_UNIT_0, PCNT_EVT_THRES_0, -Max_Pos);
    //pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    //pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);

    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_UNIT_0);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_0);
}

//Pulse Counter Direction
void pcnt_set_direction(uint8_t dir){
  if(dir){
    pcnt_set_mode(PCNT_UNIT_0, PCNT_CHANNEL_0, PCNT_COUNT_DEC, PCNT_COUNT_DEC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
  }
  else if(!dir){
    pcnt_set_mode(PCNT_UNIT_0, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
  }
}

void motor_forward(){
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 80);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void motor_backward(){
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 80);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void motor_stop(){
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B); 
}

//UART_Task
static const int RX_BUF_SIZE = 1024;

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    uint8_t Have_Goal = 0;
    int16_t Position;
    portBASE_TYPE res;
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            if(0 == strcmp("pos", (char *)data)){
                pcnt_get_counter_value(PCNT_UNIT_0, &Position);
                ESP_LOGI(MOTOR_TASK_TAG, "Position = %d", Position);
            }
            if(!Have_Goal){
                if(0 == strcmp("stop", (char *)data)){
                    ESP_LOGI(MOTOR_TASK_TAG, "Stop");
                    motor_stop();
                }
                else if(0 == strcmp("forward", (char *)data)){
                    pcnt_set_direction(0);
                    ESP_LOGI(MOTOR_TASK_TAG, "Forward");
                    motor_forward();
                }
                else if(0 == strcmp("backward", (char *)data)){
                    pcnt_set_direction(1);
                    ESP_LOGI(MOTOR_TASK_TAG, "Backward");
                    motor_backward();
                }
                else if(0 == strcmp("mid", (char *)data)){
                    ESP_LOGI(MOTOR_TASK_TAG, "Mid");
                    pcnt_get_counter_value(PCNT_UNIT_0, &Position);
                    if(Position > 0){
                        pcnt_set_direction(1);
                        ESP_LOGI(MOTOR_TASK_TAG, "Backward to Mid");
                        pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);
                        Have_Goal = 1;
                        motor_backward();
                    }
                    else if(Position < 0){
                        pcnt_set_direction(0);
                        ESP_LOGI(MOTOR_TASK_TAG, "Forward to Mid");
                        pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);
                        Have_Goal = 1;
                        motor_forward();  
                    }
                }
                else if(0 == strcmp("max", (char *)data)){
                    ESP_LOGI(MOTOR_TASK_TAG, "Max");
                    pcnt_get_counter_value(PCNT_UNIT_0, &Position);
                    if(Position < Max_Pos){
                        pcnt_set_direction(0);
                        ESP_LOGI(MOTOR_TASK_TAG, "Forward to Max");
                        pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_1);
                        Have_Goal = 1;
                        motor_forward();
                    }
                }
                else if(0 == strcmp("min", (char *)data)){
                    ESP_LOGI(MOTOR_TASK_TAG, "Min");
                    pcnt_get_counter_value(PCNT_UNIT_0, &Position);
                    if(Position > - Max_Pos){
                        pcnt_set_direction(1);
                        ESP_LOGI(MOTOR_TASK_TAG, "Backword to Min");
                        pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_THRES_0);
                        Have_Goal = 1;
                        motor_backward();
                    }
                }
                else if(0 == strcmp("clear", (char *)data)){
                    int16_t Old_Position = Position;
                    ESP_LOGI(MOTOR_TASK_TAG, "Clear start");
                    pcnt_intr_disable(PCNT_UNIT_0);
                    motor_backward();
                    while(1){
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                        pcnt_get_counter_value(PCNT_UNIT_0, &Position);
                        if(Old_Position == Position){
                            motor_stop();
                            pcnt_counter_pause(PCNT_UNIT_0);
                            pcnt_counter_clear(PCNT_UNIT_0);
                            pcnt_counter_resume(PCNT_UNIT_0);
                            ESP_LOGI(MOTOR_TASK_TAG, "Position = %d", Position);
                            ESP_LOGI(MOTOR_TASK_TAG, "Back to the shortest");
                            break;
                        }
                        Old_Position = Position;
                    }
                    motor_forward();
                    pcnt_set_direction(0);
                    while(1){
                        vTaskDelay(1);
                        pcnt_get_counter_value(PCNT_UNIT_0, &Position);
                        if(Position >= Mid_Pos){
                            motor_stop();
                            ESP_LOGI(MOTOR_TASK_TAG, "Position = %d", Position);
                            pcnt_counter_pause(PCNT_UNIT_0);
                            pcnt_counter_clear(PCNT_UNIT_0);
                            pcnt_counter_resume(PCNT_UNIT_0);
                            pcnt_intr_enable(PCNT_UNIT_0);
                            ESP_LOGI(MOTOR_TASK_TAG, "Cleared");
                            break;
                        }
                    }
                }
            }
            else{
                pcnt_evt_t evt;
                res = xQueueReceive(pcnt_evt_queue, &evt, 100 / portTICK_PERIOD_MS);
                if(res == pdTRUE){
                    if(evt.status & PCNT_EVT_THRES_1){
                        ESP_LOGI(MOTOR_TASK_TAG, "Arrived at MAX");
                        pcnt_event_disable(PCNT_UNIT_0, PCNT_EVT_THRES_1);
                        Have_Goal = 0;
                    }
                    else if(evt.status & PCNT_EVT_THRES_0){
                        ESP_LOGI(MOTOR_TASK_TAG, "Arrived at MIN");
                        pcnt_event_disable(PCNT_UNIT_0, PCNT_EVT_THRES_0);
                        Have_Goal = 0;
                    }
                    else if(evt.status & PCNT_EVT_ZERO){
                        ESP_LOGI(MOTOR_TASK_TAG, "Arrive at MID");
                        pcnt_event_disable(PCNT_UNIT_0, PCNT_EVT_ZERO);
                        Have_Goal = 0;
                    }
                }
            }

        }
        vTaskDelay(10);
    }
}

void app_main(void)
{
    esp_log_level_set(MOTOR_TASK_TAG, ESP_LOG_INFO);
    motor_init();
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_init();
    uart_init();
    gpio_set_direction(PCNT_B_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(PCNT_B_pin, 1);

    ESP_LOGI(MOTOR_TASK_TAG, "Initialized");
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}

