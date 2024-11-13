/*! @mainpage Proyecto final - 
 *
 * @section genDesc General Description
 *
 *  Este proyecto consiste desarrollar un sistema para medir y monitorear un brazo en rehabilitación, mediante un umbral de EMG 
 *  y evaluando la fuerza muscular del paciente en el tiempo.
 *
 *
 * @section hardConn Hardware Connection
 *
 * |    Peripheral  |   ESP32   	|
 * |:--------------:|:--------------|
 * |   HX711 CLK    | 	 GPIO_18	|
 * |   HX711 DATA 	| 	 GPIO_9		|
 * |   BUZZER	 	| 	 GPIO_19	|
 * |   NEOPIXEL 	| 	 GPIO_20	|
 * |   ADC CH1   	| 	 GPIO_01	|
 * |   GND	     	| 	  GND		|
 * |   +5V  	 	| 	  +5V		|
 *
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 17/10/2024 | Document creation		                         |
 *
 * @author Alvaro Ojeda (alvaro.ojeda@ingenieria.uner.edu.ar)
 * @author Giovanni Giorgio (giovanni.giorgio@ingenieria.uner.edu.ar)
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
#include "analog_io_mcu.h"
#include "uart_mcu.h"
#include "timer_mcu.h"
#include "iir_filter.h"
#include "hx711.h"

/*==================[macros and definitions]=================================*/

/** @def N_LEDS
 * @brief Numero de leds en la tira
 */
#define N_LEDS 8 

/** @def SAMPLE_FREQ
 * @brief Frecuencia de muestreo
 */
#define SAMPLE_FREQ 200 //---> ver esto xq puede que haya dado error por esto jaja deberia ser 1000?

/** @def T_SENIAL
 * @brief ??
 */
#define T_SENIAL 1000

/** @def CHUNK
 * @brief Tamaño del bloque de datos a adquirir y procesar
 */
#define CHUNK 4

/** @def T_BUZZER
 * @brief Frecuencia del buzzer
 */
#define T_BUZZER 1000 

/*==================[internal data definition]===============================*/

/** @def emg_chunk
 * @brief Bloque de n-CHUNK datos
 */
float emg_chunk[CHUNK];

/** @def emg_filtrado
 * @brief Bloque de n-CHUNK datos filtrados
 */
float emg_filtrado[CHUNK];

/** @def umbral
 * @brief Valor umbral del EMG
 */
float umbral = 100.0;

/** @def emg_task_handle
 */
TaskHandle_t emg_task_handle = NULL;

/** @def alert_task_handle
 */
TaskHandle_t alert_task_handle = NULL;

/*==================[internal functions declaration]=========================*/

/**  @def void FuncTimerSenial(void* param)
 * @brief Función invocada en la interrupción del timer A (señal)
 * @param[in] param void* que corresponde a los parametros de la tarea
 */
void FuncTimerSenial(void *param)
{
    xTaskNotifyGive(emg_task_handle);
}

/**  @def void FuncTimerAlert(void* param)
 * @brief Función invocada en la interrupción del timer B (alerta)
 * @param[in] param void* que corresponde a los parametros de la tarea
 */
void FuncTimerAlert(void *param)
{
    //xTaskNotifyGive(alert_task_handle); //Este avisa en la otra tarea
}

/**  @def void AplicarFiltrado(float *emg_entrada, float *emg_salida, uint8_t tamanio_senial)
 * @brief Función que aplica el filtro pasa bajo a los datos de la señal
 * @param[in] emg_entrada float* que corresponde a los datos de la señal
 * @param[in] emg_salida float* que corresponde a los datos filtrados
 * @param[in] tamanio_senial uint8_t que corresponde al tamaño del arreglo de datos
 */
void AplicarFiltrado(float *emg_entrada, float *emg_salida, uint8_t tamanio_senial)
{
    float senialRectificada[tamanio_senial]; // Arreglo temporal para almacenar la señal rectificada

    // Paso 1: Rectificación de la señal (tomamos el valor absoluto)
    for (uint16_t i = 0; i < tamanio_senial; ++i)
    {
        senialRectificada[i] = fabs(emg_entrada[i]);
    }

    // Paso 2: Aplicar el filtro pasa bajos a la señal rectificada
    LowPassFilter(senialRectificada, emg_salida, CHUNK);
}

/**  @def void ControlMovLEDS(uint8_t indice)
 * @brief Función que controla el movimiento de los LEDs
 * @param[in] indice uint8_t que corresponde al indice del arreglo de datos
 */
void ControlMovLEDS(uint8_t indice)
{
    float porcentaje = indice / umbral;

    if (porcentaje > 1)
    {
        porcentaje = 1;
    }

    float LedsActivos = roundf(porcentaje * N_LEDS); 

    NeoPixelAllColor(0);

    for (uint16_t pos_led = 0; pos_led < LedsActivos; pos_led++)
    {
        NeoPixelSetPixel(pos_led, NEOPIXEL_COLOR_BLUE);
    }
}

/**  @def void MedirFuerza(void)
 * @brief Función que mide la fuerza aplicada
 */
void MedirFuerza(void)
{
    float fuerza = 0;

    fuerza = HX711_get_units(5)/1000;
    printf("Valor de  Fuerza en kg Fuerza: %f", fuerza);
    printf("\r\n");
}

/**  @def void BuzzerLedTASK(void *pvParameter)
 * @brief Tarea encargada de controlar el buzzer y los LEDs
 * @param[in] pvParameter void* que corresponde a los parametros de la tarea
 */
void BuzzerLedTask(void *pvParameter)
{
    uint8_t indice = 0;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // La tarea espera en este punto hasta recibir una notificación
        
        for (uint8_t k = 0; k < CHUNK; k++) // Recorre emg_filtrado
        {
            ControlMovLEDS(emg_filtrado[k]);
            
            if(emg_filtrado[k] > umbral)
            {
                BuzzerPlayTone(NOTE_A6,300);
                NeoPixelAllColor(NEOPIXEL_COLOR_RED);                     
                MedirFuerza();
            }
        } 
    }
}

/**  @def static void EMGTask(void *pvParameter)
 * @brief Tarea encargada de adquirir la señal de la EMG
 * @param[in] pvParameter void* que corresponde a los parametros de la tarea
 */
static void EMGTask(void *pvParameter)
{
    uint16_t valor_emg = 0; // Variable para almacenar el valor del EMG
    uint8_t contador = 0; // Variable para contar el número de datos recibidos

    char msg[128]; // Variable para almacenar todo el EMG
    char msg_chunk[24]; //Variable que almacena un bloque de n=CHUNK datos del EMG

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // La tarea espera en este punto hasta recibir una notificación

        if (contador < CHUNK) // Si el contador es menor que el tamaño del bloque de datos sigo adquiriendo datos
        {
            AnalogInputReadSingle(CH1, &valor_emg); // Adquiere el valor del EMG
            emg_chunk[contador] = valor_emg; // Almacena el valor en el arreglo de datos
            contador++; // Incrementa el contador
        }
        else
        {   
            HiPassFilter(emg_chunk, emg_filtrado, CHUNK); // Filtra el bloque de n=CHUNK datos
            AplicarFiltrado(emg_filtrado, emg_filtrado, CHUNK);
            
            strcpy(msg, ""); // Limpia el contenido de la variable msg

			// Envía el chunk procesado por puerto serie
			for (uint8_t k = 0; k < CHUNK; k++) // Recorre emg_filtrado
			{
				sprintf(msg_chunk, "%.2f\r\n", emg_filtrado[k]); // Castea el valor emg_filtrado[k] en un string y lo almacena en msg_chunk
				strcat(msg,msg_chunk); // Concatena el string msg_chunk al string msg
			}
			printf(msg); // Muestra el string msg en la consola
            
            contador = 0; // Reinicia el contador
            xTaskNotifyGive(alert_task_handle);//Esto es para que notifique cuando tenga los 4 valores
        }
    }
}

/*==================[external functions definition]==========================*/
void app_main(void){

    /*Variable declarations*/
    printf("Hola! Bienvenido al sistema de seguimiento de rehabilitación muscular. \r\n");
    static neopixel_color_t color[N_LEDS];

    /*Inicializations*/
    LowPassInit(SAMPLE_FREQ, 500, ORDER_2); // Inicializo el filtro pasa bajo
    HiPassInit(SAMPLE_FREQ, 0.1, ORDER_2); // Inicializo el filtro pasa alto
    HX711_Init(128, GPIO_18, GPIO_9); // Inicializa el sensor de fuerza HX711
    BuzzerInit(GPIO_19); // Inicializo el buzzer
    NeoPixelInit(GPIO_20, N_LEDS, color); // Inicializo la tira LED

    analog_input_config_t config_senial_emg = { // Configuración del ADC para adquirir la señal de EMG
        .input = CH1,
        .mode = ADC_SINGLE,
        .func_p = NULL,
        .param_p = NULL,
        .sample_frec = 0
    };
    AnalogInputInit(&config_senial_emg); // Inicializa el ADC

    timer_config_t timer_senial = { // Configuración del timer para la adquisición de la señal de EMG
        .timer = TIMER_A,
        .period = T_SENIAL,  //Fijarme si tiene que ser T_SENIAL o T_SENIAL*CHUNK
        .func_p = FuncTimerSenial,
        .param_p = NULL
    };
    TimerInit(&timer_senial); // Inicializa el timer A

    timer_config_t timer_alerta = { // Configuración del timer para el buzzer
        .timer = TIMER_B,
        .period = T_BUZZER,
        .func_p = FuncTimerAlert,
        .param_p = NULL
    };
    TimerInit(&timer_alerta); // Inicializa el timer B
    
    /*HX711 calibration*/
    HX711_tare(5);
    HX711_tare(5); // Tara el 0 en el sensor
    HX711_setScale(2.436); // Calibrado con 100g
    printf("Por favor aguarde mientras la celda de carga es calibrada. \r\n");
    printf("Calibracion Realizada \r\n");
    NeoPixelAllOff();
    NeoPixelAllColor(NEOPIXEL_COLOR_GREEN);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    NeoPixelAllOff();

    /*Tasks*/
    xTaskCreate(EMGTask, "EMG", 4096, NULL, 5, &emg_task_handle);
    xTaskCreate(BuzzerLedTask, "Buzzer", 2048, NULL, 5, &alert_task_handle);

    /*Timers start*/
    TimerStart(timer_senial.timer);
    TimerStart(timer_alerta.timer);

}

// Posible tono para el umbral alerta:d=8,o=5,b=150:c6,e6,g6
/*==================[end of file]============================================*/
