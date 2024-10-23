/*! @mainpage Ejemplo Bluetooth - Filter
 *
 * @section genDesc General Description
 *
 * This section describes how the program works.
 *
 * <a href="https://drive.google.com/...">Operation Example</a>
 *
 * @section hardConn Hardware Connection
 *
 * |    Peripheral  |   ESP32   	|
 * |:--------------:|:--------------|
 * | 	PIN_X	 	| 	GPIO_X		|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 12/09/2023 | Document creation		                         |
 *
 * @author Alvaro Ojeda (alvaro.ojeda@ingenieria.uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "buzzer.h"
#include "led.h"
#include "neopixel_stripe.h"
#include "ble_mcu.h"
#include "analog_io_mcu.h"
#include "uart_mcu.h"
#include "timer_mcu.h"
#include "iir_filter.h"
#include "hx711.h"

/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD 500
#define N_LEDS 8 //Para la tira de leds fijarme el anillo
#define BUFFER_SIZE 256
#define SAMPLE_FREQ 200
#define T_SENIAL 1000
#define CHUNK 4
#define T_BUZZER 500*1000
/*==================[internal data definition]===============================*/

float senialPrueba[] = {
     25,28,36,55,59,78,  85,  78,  76,  85,  93,  85,  79,
     86,  93,  93,  85,  87,  94,  98,  93,  87,  95, 104,  99,  91,
     93, 102, 104,  99,  96, 101, 106, 102,  96,  97, 104, 106,  97,
     94, 100, 103, 101,  91,  95, 103, 100,  94,  90,  98, 104,  94,
     87,  93,  99,  97,  87,  86,  96,  98,65,63,62,61,60,55,54,49,20,16
};

int senialPrueba2[]={10,20,30,40,50,60,70,80,90,100,101,105,110,120,112,96,95,90,80,70,60,50,40};

volatile float emg_chunk[CHUNK];

volatile float emg_filtrado[CHUNK];

TaskHandle_t emg_task_handle = NULL;
TaskHandle_t alert_task_handle = NULL;
float umbral = 100.0;
uint32_t escala = 1;


/*==================[internal functions declaration]=========================*/

void FuncTimerSenial(void *param)
{
    //xTaskNotifyGive(emg_task_handle);
}

void FuncTimerAlert(void *param)
{
    xTaskNotifyGive(alert_task_handle);
}

void EnviarDatosUART(float *datos)
{
    for (uint8_t indice = 0; indice < CHUNK; indice++)
    {
        UartSendString(UART_PC, (char *)UartItoa(datos[indice], 10));
        UartSendString(UART_PC, "\r\n");
    }
}

void AplicarFiltrado(float *emg_entrada, float *emg_salida, uint8_t tamanio_senial)
{
    float senialRectificada[tamanio_senial]; // Arreglo temporal para almacenar la señal rectificada

    // Paso 1: Rectificación de la señal (tomamos el valor absoluto)
    for (uint16_t i = 0; i < tamanio_senial; ++i)
    {
        senialRectificada[i] = fabs(emg_entrada[i]);
    }

    // Paso 2: Aplicar el filtro pasa bajos a la señal rectificada
    LowPassFilter(emg_entrada, emg_salida, CHUNK);
}


void ControlMovLEDS(void){
        //de ultima si no funca a la mierda
    
    uint8_t indice=0;
    float porcentaje= senialPrueba2[indice]/umbral;
    float LedsActivos = roundf(porcentaje*N_LEDS);//Casteo a int(?)
    NeoPixelAllOff();
    for(uint16_t pos_led=0;pos_led<LedsActivos;pos_led++){
        
        NeoPixelSetPixel(pos_led,NEOPIXEL_COLOR_BLUE);
    }
    
}

void CalibrarCeldaCarga(float pesoReal){
    HX711_tare(10);
    uint32_t lecturaPeso = HX711_read();//De ultima HX711_readAverage(10);
    float escala = lecturaPeso/pesoReal;
    HX711_setScale(escala);


}

void MedirFuerzaTASK(void *pvParameter){
    uint8_t indice=0;
    float fuerza=0;
    while(true){

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(senialPrueba[indice] >= umbral)
        {
        //HX711tare(10); no se si va en el main o aca
        fuerza = HX711_get_units(10);
        }

    }
}

void BuzzerLedTASK(void *pvParameter)
{

    uint8_t indice=0;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(indice < 23)
        {
            if(senialPrueba2[indice] > umbral)
            {
                
                //BuzzerPlayTone(NOTE_A6,250);
                NeoPixelAllColor(NEOPIXEL_COLOR_RED);
                //printf("%d",senialPrueba2[indice]); esto no me funcionaba no se por que
                //printf("\r\n");

            }
            //ControlMovLEDS();
            indice++;
        }else if (indice == 23)
		{

			indice = 0;
		}

    }
}
static void EMGTask(void *pvParameter)
{
    uint16_t valor_emg = 0;
    uint8_t contador = 0;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (contador < CHUNK)
        {
            AnalogInputReadSingle(CH1, &valor_emg);
            emg_chunk[contador] = valor_emg;
            contador++;
        }
        else
        {
            AplicarFiltrado(emg_chunk, emg_filtrado, CHUNK);
            //si encuentra el umbral que mida la fuerza de la mano, emite alerta
            EnviarDatosUART(emg_filtrado);//para ver los datos
            
            contador = 0;
        }
        
    }
}
/*==================[external functions definition]==========================*/
void app_main(void)
{
    static neopixel_color_t color[N_LEDS];
    analog_input_config_t config_senial_emg = {
        .input = CH1,
        .mode = ADC_SINGLE,
        .func_p = NULL,
        .param_p = NULL,
        .sample_frec = 0
    };

    timer_config_t timer_senial = {
        .timer = TIMER_A,
        .period = T_SENIAL,
        .func_p = FuncTimerSenial,
        .param_p = NULL
    };

    timer_config_t timer_alerta = {
        .timer = TIMER_B,
        .period = T_BUZZER,
        .func_p = FuncTimerAlert,
        .param_p = NULL
    };

    //inicializacion de input,filtro Pasa bajo, timer, buzzer
    AnalogInputInit(&config_senial_emg);
    LowPassInit(SAMPLE_FREQ, 30, ORDER_2);

    TimerInit(&timer_senial);
    TimerInit(&timer_alerta);
    BuzzerInit(GPIO_9);
    NeoPixelInit(GPIO_20, N_LEDS, color);
    NeoPixelAllOff();
    //BuzzerOn();

    //SCL: GPIO_7
    //SDA: GPIO_6
    HX711_Init(128,GPIO_7, GPIO_6);
    //CalibrarCeldaCarga(200);

    //HX711tare(10);

    //HX711_setScale();//No se que valor va aca
    
    //creacion de tareas y arranca el timer
    //xTaskCreate(EMGTask, "EMG", 4096, NULL, 5, &emg_task_handle);
    xTaskCreate(BuzzerLedTASK, "Buzzer", 1024, NULL, 5, &alert_task_handle);
    //xTaskCreate(MedirFuerzaTASK, "MedirFuerza", 1024, NULL, 5, &emg_task_handle);

    //TimerStart(timer_senial.timer);
    TimerStart(timer_alerta.timer);











    /*
    uint8_t blink = 0;
    static neopixel_color_t color;
    ble_config_t ble_configuration = {
        "ESP_EDU_1",
        read_data
    };
    timer_config_t timer_senial = {
        .timer = TIMER_B,
        .period = T_SENIAL*CHUNK,
        .func_p = FuncTimerSenial,
        .param_p = NULL
    };

    NeoPixelInit(BUILT_IN_RGB_LED_PIN, BUILT_IN_RGB_LED_LENGTH, &color);
    NeoPixelAllOff();
    TimerInit(&timer_senial);
    LedsInit();
    LowPassInit(SAMPLE_FREQ, 30, ORDER_2);
    HiPassInit(SAMPLE_FREQ, 1, ORDER_2);
    BleInit(&ble_configuration);

    xTaskCreate(EMGTask, "FFT", 4096, NULL, 5, &emg_task_handle);
    TimerStart(timer_senial.timer);

    while(1){
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        switch(BleStatus()){
            case BLE_OFF:
                NeoPixelAllOff();
            break;
            case BLE_DISCONNECTED:
                if(blink%2){
                    NeoPixelAllColor(NEOPIXEL_COLOR_BLUE);
                }else{
                    NeoPixelAllOff();
                }
                blink++;
            break;
            case BLE_CONNECTED:
                NeoPixelAllColor(NEOPIXEL_COLOR_BLUE);
            break;
        }
    }*/
}

//Posible tono para el umbral alerta:d=8,o=5,b=150:c6,e6,g6
/*==================[end of file]============================================*/
