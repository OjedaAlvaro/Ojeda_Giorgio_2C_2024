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
 * @author Albano Peñalva (albano.penalva@uner.edu.ar)
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"
#include "neopixel_stripe.h"
#include "ble_mcu.h"
#include "analog_io_mcu.h"
#include "uart_mcu.h"
#include "timer_mcu.h"

#include "iir_filter.h"
/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD 500
#define LED_BT	            LED_1
#define BUFFER_SIZE         256
#define SAMPLE_FREQ	        200
#define T_SENIAL            1000 
#define CHUNK               4 
#define DELAY_MUESTREO_EMG 1 * 1000
/*==================[internal data definition]===============================*/

static float ecg_filt[CHUNK];
TaskHandle_t emg_task_handle = NULL;
//bool filter = false;

float emg_filt[CHUNK];

/*==================[internal functions declaration]=========================*/


void FuncTimerSenial(void* param){
    xTaskNotifyGive(emg_task_handle);
}


static void EMGTask(void *pvParameter)
{
	uint16_t senial_analogica = 0;
    uint16_t chunk[CHUNK];
    uint8_t contador = 0;
	while (true)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(contador < CHUNK)
        {
            AnalogInputReadSingle(CH1, &senial_analogica);
            emg_filt[contador] = senial_analogica;
            contador++;

        }else 
        {   UartSendString(UART_PC, (char *)UartItoa(senial_analogica, 10));
	        UartSendString(UART_PC, "\r\n");
            contador = 0;
        }
        //aca deberia ir una funcion de filtrado
        LowPassFilter(emg_filt, emg_filt, CHUNK);
		
	}
}
/*==================[external functions definition]==========================*/
void app_main(void){

	analog_input_config_t config_senial_emg = {
		.input = CH1,
		.mode = ADC_SINGLE,
		.func_p = NULL,
		.param_p = NULL,
		.sample_frec = 0

	};
    AnalogInputInit(&config_senial_emg);
    LowPassInit(SAMPLE_FREQ, 30, ORDER_2);


    timer_config_t timer_senial = {
    .timer = TIMER_B,
    .period = T_SENIAL,
    .func_p = FuncTimerSenial,
    .param_p = NULL
    };

    TimerInit(&timer_senial.timer);
    
    xTaskCreate(EMGTask, "EMG", 4096, NULL, 5, &emg_task_handle);
    TimerStart(timer_senial.timer);


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

/*==================[end of file]============================================*/
