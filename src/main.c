// ******************** Include libs C ******************** //

#include <stdio.h>
#include "string.h"

// ******************** Include libs do FreeRTOS ******************** //
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include"freertos/timers.h"
#include "freertos/semphr.h"

// ******************** Include libs de acesso ao hardware ******************** //

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"

// ******************** Defines para funções de hardware ******************** //

#define LED_G 17
#define LED_R 16
#define LED 2
#define button 15
#define OUTPUT_MASK ((1<<LED_G)|(1<<LED_R)|(1<<LED))
#define INPUT_MASK ((1 << button))
#define HIGH 1
#define LOW 0

// ******************** Define para processamento de tasks ******************** //

#define HTTP_REQUEST_PROC_LOAD 0xfffff
#define RS232_CHAR_PROC_LOAD 0xfffff
#define Control 0xffff
#define CPU_limit (float)(85)

// ******************** Constantes de tempo para timer ******************** //

#define xTimerReadSensorTick 50
#define xTimerControlTick 100
#define xTimerTaskCPUTick 50
#define xTimerTaskHTTPTick 500
#define xTimerTaskRS232Tick 500
#define xTimerTaskLedTick 1050
#define xTimerTaskKeyscanTick 1000
#define xTimerNoWait 0

// ******************** Buffer para uart ******************** //
#define BUF_SIZE (1024)

// ******************** Handle para as tasks ******************** //

TaskHandle_t taskreadsensor = NULL; 
TaskHandle_t taskcontrol = NULL; 
TaskHandle_t taskCPU = NULL;
TaskHandle_t taskHTTP = NULL;
TaskHandle_t taskRS232 = NULL;
TaskHandle_t task_led_control = NULL;
TaskHandle_t KeyScanTask = NULL;


// ******************** Handle da fila ******************** //

QueueHandle_t xQueueSensores;

// ******************** Handle dos timers ******************** //
TimerHandle_t xTimerReadSensor;
TimerHandle_t xTimerControl;
TimerHandle_t xTimerTaskCPU;
TimerHandle_t xTimerTaskHTTP;
TimerHandle_t xTimerTaskRS232;
TimerHandle_t xTimerTaskLed;
TimerHandle_t xTimerTaskKeyscan;

// ******************** Variaveis globais ******************** //

float CPU = 0.0;
TickType_t idletime = 0;
TickType_t LastTime = 0;
int g_value_sensor1;
int g_value_sensor2;


// ******************** Declaração da função de configuração de GPIO ******************** //
void gpio_all_config();

// ******************** Declaração das função das tasks ******************** //
void Task_Read_Sensor(void *pvParameters); 
void task_control(void *pvParameters);
void CPU_usage(void *pvParameters);
void HTTP(void *pvParameters);
void RS232Task(void*pvParameters);
void Led_Control(void *pvParameters);
void KeyScan(void *pvParameters);


// ******************** Declaração das funções de callback dos timers ******************** //
void ReadCallback(TimerHandle_t xTimer);
void ControlCallback(TimerHandle_t xTimer);
void CPUCallback(TimerHandle_t xTimer);
void HTTPCallback(TimerHandle_t xTimer);
void RS232Callback(TimerHandle_t xTimer);
void LedCallback(TimerHandle_t xTimer);
void KeyScanCallback(TimerHandle_t xTimer);


// ******************** Estrutura para configuração de GPIO output e input  ******************** //
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



// ******************** Função de configuração de GPIO e ADC ******************** //
void gpio_all_config(){
 gpio_config(&gpio_output);
 gpio_config(&gpio_input);

/********************************************************************/
 adc1_config_width(ADC_WIDTH_BIT_12);
 adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);
 adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_0);
}

// ******************** Estrutura para configuração do uart ******************** //
const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };


void app_main() {
// ******************** Chamada função de configuração de GPIO e ADC ******************** //
 gpio_all_config();
 
 // ******************** Configuração do uart ******************** //
 uart_param_config(uart_num, &uart_config);
 uart_set_pin(uart_num,1,3,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
 uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE, 0, NULL, 0);
 
 // ******************** Criação das tasks ******************** //
 xTaskCreate(Task_Read_Sensor,"Task Read Sensor",configMINIMAL_STACK_SIZE+1024,NULL,2,&taskreadsensor);
 xTaskCreate(task_control,"Task Control",configMINIMAL_STACK_SIZE*2,NULL,3,&taskcontrol);
 xTaskCreate(CPU_usage,"CPU Usage",configMINIMAL_STACK_SIZE+1024,NULL,2,&taskCPU);
 xTaskCreate(HTTP,"HTTP Task",configMINIMAL_STACK_SIZE,NULL,1,&taskHTTP);
 xTaskCreate(RS232Task,"RS232 Task",configMINIMAL_STACK_SIZE,NULL,1,&taskRS232);
 xTaskCreate(Led_Control,"Led Control",configMINIMAL_STACK_SIZE+1024,NULL,1,&task_led_control);
 xTaskCreate(KeyScan,"Key Scan Task",configMINIMAL_STACK_SIZE+2048,NULL,2,&KeyScanTask);
 
// ******************** Criação dos timers ******************** //
 xTimerTaskHTTP = xTimerCreate("HTTP TIMER",pdMS_TO_TICKS(xTimerTaskHTTPTick),pdFALSE,0,HTTPCallback);
 xTimerTaskRS232 = xTimerCreate("RS232 Timer",pdMS_TO_TICKS(xTimerTaskRS232Tick),pdFALSE,0,RS232Callback);
 xTimerTaskKeyscan = xTimerCreate("Key Timer",pdMS_TO_TICKS(xTimerTaskKeyscanTick),pdFALSE,0,KeyScanCallback);
 xTimerTaskCPU = xTimerCreate("CPU Timer",pdMS_TO_TICKS(xTimerTaskCPUTick),pdFALSE,0,HTTPCallback);
 xTimerTaskLed = xTimerCreate("LED Timer",pdMS_TO_TICKS(xTimerTaskLedTick),pdFALSE,( void * ) 0,LedCallback);
 xTimerReadSensor = xTimerCreate("ReadSensor Timer",pdMS_TO_TICKS(xTimerReadSensorTick),pdFALSE,0,ReadCallback);
 xTimerControl = xTimerCreate("Control Timer",pdMS_TO_TICKS(xTimerControlTick),pdFALSE,0,ControlCallback);
 

 // ******************** Criação da fila ******************** //
 xQueueSensores = xQueueCreate(2,sizeof(int));

}

// ******************** Task responsável por realizar as leituras dos sensores ******************** //
void Task_Read_Sensor(void* pvParameters){

    int value_sensor1 = 0;
    int value_sensor2 = 0;
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    xLastWakeTimer = xTaskGetTickCount();
    TickType_t evaluate_premp;

    while(1){
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        evaluate_premp = xFrequency -(xTaskGetTickCount()-xLastWakeTimer);
        xTimerChangePeriod(xTimerReadSensor,(pdMS_TO_TICKS(xTimerReadSensorTick) - evaluate_premp),pdMS_TO_TICKS(0));
        xTimerStart(xTimerReadSensor,pdMS_TO_TICKS(xTimerNoWait));
        value_sensor1 = adc1_get_raw(ADC2_CHANNEL_0);
        value_sensor2 = adc1_get_raw(ADC2_CHANNEL_3);
        xQueueSend(xQueueSensores,&value_sensor1,portMAX_DELAY);
        xQueueSend(xQueueSensores,&value_sensor2,portMAX_DELAY);
        xTimerStop(xTimerReadSensor,pdMS_TO_TICKS(xTimerNoWait));
     
    }    
}

// ******************** Task responsável por processar os dados recebidos ******************** //

void task_control(void *pvParameter){
    int sensor1 = 0;
    int sensor2 = 0;
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    xLastWakeTimer = xTaskGetTickCount();
    TickType_t evaluate_premp;
    int i;
    while(1){
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        evaluate_premp = xFrequency -(xTaskGetTickCount()-xLastWakeTimer);
        xTimerChangePeriod(xTimerControl,(pdMS_TO_TICKS(xTimerControlTick) - evaluate_premp),pdMS_TO_TICKS(0));
        xTimerStart(xTimerControl,pdMS_TO_TICKS(xTimerNoWait));
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
                xTaskNotifyGive(taskHTTP);
                xTaskNotifyGive(taskRS232);
                
            
            }
        
            
        }
        xTimerStop(xTimerControl,pdMS_TO_TICKS(xTimerNoWait));
    }

}


// ******************** Task responsável por controlar os leds ******************** //
void Led_Control(void *pvParameters){
    while(1){
        //xTimerStart(xTimerTaskLed,pdMS_TO_TICKS(xTimerNoWait));
        if(CPU <= CPU_limit){
            gpio_set_level(LED_G,LOW);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_G,HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
        }else if(CPU > CPU_limit){
            gpio_set_level(LED_R,LOW);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_R,HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        //xTimerStop(xTimerTaskLed,pdMS_TO_TICKS(xTimerNoWait));
        vTaskDelay(pdMS_TO_TICKS(50));
        
        
    }
    
}

// ******************** Task responsável por realizar uma verificação do tempo de uso da CPU ******************** //

void CPU_usage(void *pvParameters){
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    xLastWakeTimer = xTaskGetTickCount();
    TickType_t evaluate_premp;
    while (1){
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        evaluate_premp = xFrequency -(xTaskGetTickCount()-xLastWakeTimer);
        xTimerChangePeriod(xTimerTaskCPU,(pdMS_TO_TICKS(xTimerTaskCPUTick) - evaluate_premp),pdMS_TO_TICKS(0));
        xTimerStart(xTimerTaskCPU,pdMS_TO_TICKS(xTimerNoWait));
        CPU = ((float)(1000) - ((float)(idletime)/(float)(CPU_CLK_FREQ)))/((float)(1000));
        idletime = 0;
        xTimerStop(xTimerTaskCPU,pdMS_TO_TICKS(xTimerNoWait));
        
        
    }
    
}


// ******************** Idle Hook ******************** //
void vApplicationIdleHook( void ){
    unsigned long int tick = xTaskGetTickCount();
    while (xTaskGetTickCount() == tick){
        idletime = idletime + 1;
    }
    
}

// ******************** Task periódica de HTTP ******************** //
void HTTP(void*pvParameters){
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    xLastWakeTimer = xTaskGetTickCount();
    TickType_t evaluate_premp;
    int i;
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        evaluate_premp = xFrequency -(xTaskGetTickCount()-xLastWakeTimer);
        xTimerChangePeriod(xTimerTaskHTTP,(pdMS_TO_TICKS(xTimerTaskHTTPTick) - evaluate_premp),pdMS_TO_TICKS(0));
        xTimerStart(xTimerTaskHTTP,pdMS_TO_TICKS(xTimerNoWait));
        for(i = 0;i < HTTP_REQUEST_PROC_LOAD ;i++ );
        xTimerStop(xTimerTaskHTTP,pdMS_TO_TICKS(xTimerNoWait));


       
    }
    
}

// ******************** Task aperiódica de RS232 ******************** //
void RS232Task(void*pvParameters){
    TickType_t xLastWakeTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    xLastWakeTimer = xTaskGetTickCount();
    TickType_t evaluate_premp;
    int i;
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTimer,xFrequency);
        evaluate_premp = xFrequency -(xTaskGetTickCount()-xLastWakeTimer);
        xTimerChangePeriod(xTimerTaskRS232,(pdMS_TO_TICKS(xTimerTaskRS232Tick) - evaluate_premp),pdMS_TO_TICKS(0));
        xTimerStart(xTimerTaskRS232,pdMS_TO_TICKS(xTimerNoWait));
        for(i = 0;i < RS232_CHAR_PROC_LOAD ;i++ );
        xTimerStop(xTimerTaskRS232,pdMS_TO_TICKS(xTimerNoWait));
    }
    
}

// ******************** Task responsável por ler dados externos do teclado ******************** //
void KeyScan(void *pvParameters){

   char* CPU_String = malloc(sizeof(char) * BUF_SIZE);
   char*  string_sensor1 = malloc(sizeof(char) * BUF_SIZE);
   char*  string_sensor2 = malloc(sizeof(char) * BUF_SIZE);
   uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while(1){
        xTimerStart(xTimerTaskKeyscan,xTimerNoWait);
        int len = uart_read_bytes(uart_num,data,1,pdMS_TO_TICKS(20));
        if(len > 0){
            data[len] = '\0';
            uart_write_bytes(uart_num,data,( uint8_t ) (1) );
            switch (data[len - 1]){
            case '1':
                sprintf(CPU_String,"- [CPU USAGE] %f%%\n\r",CPU);
                uart_write_bytes(uart_num,CPU_String,strlen(CPU_String));
                break;
            case '2':
                sprintf(string_sensor1," - [Sensor 1] %d\n\r",g_value_sensor1);
                uart_write_bytes(uart_num,string_sensor1,strlen(string_sensor1));
                break;
            case '3':
                sprintf(string_sensor2," - [Sensor 2] %d\n\r",g_value_sensor2);
                uart_write_bytes(uart_num,string_sensor2,strlen(string_sensor2));
                break;
            default:
                uart_write_bytes(uart_num,"\e[1;1H\e[2J 1 - CPU USAGE\n\r2 - Sensor 1 value \n\r3 - Sensor 2 value\n\r",strlen("\e[1;1H\e[2J 1 - CPU USAGE\n\r2 - Sensor 1 value \n\r3 - Sensor 2 value\n\r"));
                break;
            }
            
            uart_flush(uart_num);
            uart_flush_input(uart_num);
            

        } 
        xTimerStop(xTimerTaskKeyscan,pdMS_TO_TICKS(xTimerNoWait));
    }
    
}

// ******************** Funções de CallBack ******************** //

 void HTTPCallback(TimerHandle_t xTimer){
    uart_write_bytes(uart_num,"Timeout HTTP task\n\r",strlen("Timeout HTTP task\n\r"));
}  

void RS232Callback(TimerHandle_t xTimer){
    uart_write_bytes(uart_num,"Timeout RS232 task\n\r",strlen("Timeout RS232 task\n\r"));
}
 
void LedCallback(TimerHandle_t xTimer){
    uart_write_bytes(uart_num,"Timeout RS232 task\n\r",strlen("Timeout LED task\n\r"));
}

void ReadCallback(TimerHandle_t xTimer){
    uart_write_bytes(uart_num,"Timeout Read task\n\r",strlen("Timeout Read task\n\r"));
}
void ControlCallback(TimerHandle_t xTimer){
    uart_write_bytes(uart_num,"Timeout Control task\n\r",strlen("Timeout Control task\n\r"));
}
void CPUCallback(TimerHandle_t xTimer){
    uart_write_bytes(uart_num,"Timeout CPU usage task\n\r",strlen("Timeout CPU usage task\n\r"));
}


void KeyScanCallback(TimerHandle_t xTimer){
    uart_write_bytes(uart_num,"Timeout Keyscan task\n\r",strlen("Timeout Keyscan task\n\r"));
}
