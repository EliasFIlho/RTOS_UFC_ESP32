#include <stdio.h>
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include"freertos/timers.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"

#define LED_G 17
#define LED_R 16
#define LED 2
#define button 15
#define OUTPUT_MASK ((1<<LED_G)|(1<<LED_R)|(1<<LED))
#define INPUT_MASK ((1 << button))
#define HIGH 1
#define LOW 0
#define HTTP_REQUEST_PROC_LOAD 0xfffff
#define RS232_CHAR_PROC_LOAD 0xfffff
#define Control 0xfff
#define CPU_limit (float)(85)


#define BUF_SIZE (1024)



TaskHandle_t taskreadsensor = NULL; 
TaskHandle_t taskcontrol = NULL; 
TaskHandle_t taskCPU = NULL;
TaskHandle_t taskHTTP = NULL;
TaskHandle_t taskRS232 = NULL;
TaskHandle_t task_led_control = NULL;
TaskHandle_t KeyScanTask = NULL;
QueueHandle_t xQueueSensores;
QueueHandle_t xQueueCPU;

TimerHandle_t xTimerReadSensor;
TimerHandle_t xTimerControl;
TimerHandle_t xTimerTaskCPU;
TimerHandle_t xTimerTaskHTTP;
TimerHandle_t xTimerTaskRS232;
TimerHandle_t xTimerTaskLed;
TimerHandle_t xTimerTaskKeyscan;

SemaphoreHandle_t xUartMutex;


float CPU = 0.0;
TickType_t idletime = 0;
TickType_t LastTime = 0;

int g_value_sensor1;
int g_value_sensor2;


void gpio_all_config();
void Task_Read_Sensor(void *pvParameters); 
void task_control(void *pvParameters);
void CPU_usage(void *pvParameters);
void HTTP(void *pvParameters);
void RS232Task(void*pvParameters);
void Led_Control(void *pvParameters);
void LedBlink1Hz(int led);
void KeyScan(void *pvParameters);



gpio_config_t gpio_output={
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = OUTPUT_MASK,
    .pull_down_en = 0,
    .pull_up_en = 0,
};

gpio_config_t gpio_input ={
    .intr_type = GPIO_INTR_POSEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = INPUT_MASK,
    .pull_down_en =0,
    .pull_up_en = 0,
};




void gpio_all_config(){
 gpio_config(&gpio_output);
 gpio_config(&gpio_input);

/********************************************************************/
 adc1_config_width(ADC_WIDTH_BIT_12);
 adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);
 adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_0);
}


const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };


void app_main() {
 gpio_all_config();
 
 uart_param_config(uart_num, &uart_config);
 uart_set_pin(uart_num,1,3,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
 uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE, 0, NULL, 0);
 
 xTaskCreate(Task_Read_Sensor,"Task Read Sensor",configMINIMAL_STACK_SIZE+1024,NULL,1,&taskreadsensor);
 xTaskCreate(task_control,"Task Control",configMINIMAL_STACK_SIZE*2,NULL,2,&taskcontrol);
 xTaskCreate(CPU_usage,"CPU Usage",configMINIMAL_STACK_SIZE+1024,NULL,1,&taskCPU);
 xTaskCreate(HTTP,"HTTP Task",configMINIMAL_STACK_SIZE,NULL,1,&taskHTTP);
 xTaskCreate(RS232Task,"RS232 Task",configMINIMAL_STACK_SIZE,NULL,1,&taskRS232);
 xTaskCreate(Led_Control,"Led Control",configMINIMAL_STACK_SIZE+1024,NULL,1,&task_led_control);
 xTaskCreate(KeyScan,"Key Scan Task",configMINIMAL_STACK_SIZE+2048,NULL,2,&KeyScanTask);
 
 
 xQueueSensores = xQueueCreate(2,sizeof(int));
 xUartMutex = xSemaphoreCreateMutex();
}

void Task_Read_Sensor(void* pvParameters){

    int value_sensor1 = 0;
    int value_sensor2 = 0;
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    xLastWakeTimer = xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        value_sensor1 = adc1_get_raw(ADC2_CHANNEL_0);
        value_sensor2 = adc1_get_raw(ADC2_CHANNEL_3);
        xQueueSend(xQueueSensores,&value_sensor1,portMAX_DELAY);
        xQueueSend(xQueueSensores,&value_sensor2,portMAX_DELAY);
    }    
}

void task_control(void *pvParameter){
    int sensor1 = 0;
    int sensor2 = 0;
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    xLastWakeTimer = xTaskGetTickCount();
    int i;
    while(1){
        
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        if(xQueueReceive(xQueueSensores,&sensor1,pdMS_TO_TICKS(11)) == pdTRUE){
            if(xQueueReceive(xQueueSensores,&sensor2,pdMS_TO_TICKS(11)) == pdTRUE){
                if(sensor1 < 150 && sensor2 > 200){
                    gpio_set_level(LED,HIGH);      
                }else{
                    gpio_set_level(LED,LOW);
                }
                
                g_value_sensor1 = sensor1;
                g_value_sensor2 = sensor2;
                for(i = 0;i < Control;i++);
            
            }
        
            
        }

    }

}

void LedBlink1Hz(int led){
    gpio_set_level(led,LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(led,HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
}

   
void Led_Control(void *pvParameters){
    while(1){
        if(CPU <= CPU_limit){
            LedBlink1Hz(LED_G);
        }else if(CPU > CPU_limit){
            LedBlink1Hz(LED_R);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
}

void CPU_usage(void *pvParameters){
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    xLastWakeTimer = xTaskGetTickCount();
    while (1){
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        CPU = ((float)(1000) - ((float)(idletime)/(float)(CPU_CLK_FREQ)))/((float)(1000));
        idletime = 0;
        
        
    }
    
}


void vApplicationIdleHook( void ){
    unsigned long int tick = xTaskGetTickCount();
    while (xTaskGetTickCount() == tick){
        idletime = idletime + 1;
    }
    
}


void HTTP(void*pvParameters){
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    xLastWakeTimer = xTaskGetTickCount();
    int i;
    while(1){
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        for(i = 0;i < HTTP_REQUEST_PROC_LOAD ;i++ );
    }
    
}


void RS232Task(void*pvParameters){
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    xLastWakeTimer = xTaskGetTickCount();
    int i;
    while(1){
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        for(i = 0;i < RS232_CHAR_PROC_LOAD ;i++ );
    }
    
}

void KeyScan(void *pvParameters){

   char* CPU_String = malloc(sizeof(char) * BUF_SIZE);
   char*  string_sensor1 = malloc(sizeof(char) * BUF_SIZE);
   char*  string_sensor2 = malloc(sizeof(char) * BUF_SIZE);
   uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while(1){
        xSemaphoreTake(xUartMutex,portMAX_DELAY);
        int len = uart_read_bytes(uart_num,data,1,pdMS_TO_TICKS(20));
        xSemaphoreGive(xUartMutex);
        if(len > 0){
            data[len] = '\0';
            uart_write_bytes(uart_num,data,( uint8_t ) (1) );
            switch (data[len - 1]){
            case '1':
                sprintf(CPU_String,"- [CPU USAGE] %f%%\n\r",CPU);
                xSemaphoreTake(xUartMutex,portMAX_DELAY);
                uart_write_bytes(uart_num,CPU_String,strlen(CPU_String));
                xSemaphoreGive(xUartMutex);
                break;
            case '2':
               
                sprintf(string_sensor1," - [Sensor 1] %d\n\r",g_value_sensor1);
                xSemaphoreTake(xUartMutex,portMAX_DELAY);
                uart_write_bytes(uart_num,string_sensor1,strlen(string_sensor1));
                xSemaphoreGive(xUartMutex);
                break;
            case '3':
                sprintf(string_sensor2," - [Sensor 2] %d\n\r",g_value_sensor2);
                xSemaphoreTake(xUartMutex,portMAX_DELAY);
                uart_write_bytes(uart_num,string_sensor2,strlen(string_sensor2));
                xSemaphoreGive(xUartMutex);
                break;
            default:
                xSemaphoreTake(xUartMutex,portMAX_DELAY);
                uart_write_bytes(uart_num,"\e[1;1H\e[2J 1 - CPU USAGE\n\r2 - Sensor 1 value \n\r3 - Sensor 2 value\n\r",strlen("\e[1;1H\e[2J 1 - CPU USAGE\n\r2 - Sensor 1 value \n\r3 - Sensor 2 value\n\r"));
                xSemaphoreGive(xUartMutex);
                break;
            }
            
            uart_flush(uart_num);
            uart_flush_input(uart_num);

        } 
        
    }
    
}


