#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
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
#define CPU_limit (float)(85)
TickType_t idletime;

TaskHandle_t taskreadsensor = NULL; 
TaskHandle_t taskcontrol = NULL; 
TaskHandle_t taskCPU = NULL;
TaskHandle_t taskHTTP = NULL;
TaskHandle_t taskRS232 = NULL;
TaskHandle_t task_led_control = NULL;
QueueHandle_t xQueueSensores;
QueueHandle_t xQueueCPU;

void gpio_all_config();
void Task_Read_Sensor(void *pvParameters); 
void task_control(void *pvParameters);
void CPU_usage(void *pvParameters);
void HTTP(void *pvParameters);
void RS232Task(void*pvParameters);
void Led_Control(void *pvParameters);
void LedBlink1Hz(int led);
gpio_config_t gpio_output={
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = OUTPUT_MASK,
    .pull_down_en =0,
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
 adc1_config_width(ADC_WIDTH_BIT_9);
 adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);
 adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_0);
}


void app_main() {
 gpio_all_config();

 xTaskCreate(Task_Read_Sensor,"Task Read Sensor",configMINIMAL_STACK_SIZE+1024,NULL,1,&taskreadsensor);
 xTaskCreate(task_control,"Task Control",configMINIMAL_STACK_SIZE*2,NULL,2,&taskcontrol);
 xTaskCreate(CPU_usage,"CPU Usage",configMINIMAL_STACK_SIZE+1024,NULL,1,&taskCPU);
 xTaskCreate(HTTP,"HTTP Task",configMINIMAL_STACK_SIZE,NULL,1,&taskHTTP);
 xTaskCreate(RS232Task,"RS232 Task",configMINIMAL_STACK_SIZE,NULL,1,&taskRS232);
 xTaskCreate(Led_Control,"Led Control",configMINIMAL_STACK_SIZE,NULL,1,&task_led_control);
 
 
 xQueueSensores = xQueueCreate(2,sizeof(int));
 xQueueCPU = xQueueCreate(1,sizeof(float));
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
    while(1){
        
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        if(xQueueReceive(xQueueSensores,&sensor1,pdMS_TO_TICKS(11)) == pdTRUE){
            if(xQueueReceive(xQueueSensores,&sensor2,pdMS_TO_TICKS(11)) == pdTRUE){
                if(sensor1 < 150 && sensor2 > 200){
                    gpio_set_level(LED,HIGH);      
                }else{
                    gpio_set_level(LED,LOW);
                }
            
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
    float CPU_U = 7;
    while(1){
        xQueueReceive(xQueueCPU,&CPU_U,(TickType_t) 0);
        if(CPU_U <= CPU_limit && CPU_U > (float)(0)){
            LedBlink1Hz(LED_G);
        }else if(CPU_U > CPU_limit && CPU_U <= (float)(100)){
            LedBlink1Hz(LED_R);
        }else{
            printf("[CPU: %f%%] \n",CPU_U);
        }
        vTaskDelay(pdMS_TO_TICKS(50));

    }
    
}

void CPU_usage(void *pvParameters){
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    xLastWakeTimer = xTaskGetTickCount();
    TickType_t CurrentTime;
    TickType_t LastTime = 0;
    TickType_t ElapsedTime;
    float CPU;
    while (1){
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        CurrentTime = xTaskGetTickCount();
        ElapsedTime = CurrentTime - LastTime;
        CPU = ((float)(ElapsedTime) - (float)(idletime))/(float)(ElapsedTime);
        LastTime = CurrentTime;
        idletime = 0;
        xQueueOverwrite(xQueueCPU,&CPU);
        printf("[CPU: %f%%] \n",CPU);
        
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
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    xLastWakeTimer = xTaskGetTickCount();
    int i;
    while(1){
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        for(i = 0;i < RS232_CHAR_PROC_LOAD ;i++ );
    }
    
}



void vApplicationIdleHook( void ){
    unsigned long int tick = xTaskGetTickCount();
    while (xTaskGetTickCount() == tick){
        idletime =+1;
    }
    
}