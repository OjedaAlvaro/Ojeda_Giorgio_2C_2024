/*! @mainpage Proyecto final - 
 *
 * @section genDesc General Description
 *
 * This section describes how the program works.
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
#include "ble_mcu.h"
#include "analog_io_mcu.h"
#include "uart_mcu.h"
#include "timer_mcu.h"
#include "iir_filter.h"
#include "hx711.h"

/*==================[macros and definitions]=================================*/
/** @def CONFIG_BLINK_PERIOD 
 * @brief 
 */
//#define CONFIG_BLINK_PERIOD 500

/** @def N_LEDS
 * @brief Numero de leds en la tira
 */
#define N_LEDS 8 // Para la tira de leds fijarme el anillo

/** @def BUFFER_SIZE
 * @brief Tamaño del buffer de datos
 */
#define BUFFER_SIZE 256

/** @def SAMPLE_FREQ
 * @brief Frecuencia de muestreo
 */
#define SAMPLE_FREQ 200

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
#define T_BUZZER 1000 * 1000
/*==================[internal data definition]===============================*/

/** @def senialPrueba
 * @brief Señal de prueba 1
 */
float senialPrueba[] = {
    25, 28, 36, 55, 59, 78, 85, 78, 76, 85, 93, 85, 79,
    86, 93, 93, 85, 87, 94, 98, 93, 87, 95, 104, 99, 91,
    93, 102, 104, 99, 96, 101, 106, 102, 96, 97, 104, 106, 97,
    94, 100, 103, 101, 91, 95, 103, 100, 94, 90, 98, 104, 94,
    87, 93, 99, 97, 87, 86, 96, 98, 65, 63, 62, 61, 60, 55, 54, 49, 20, 16};

/** @def senialPrueba2
 * @brief Señal de prueba 2
 */
int senialPrueba2[] = {
    10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 101, 105,
    110, 120, 112, 96, 95, 90, 80, 70, 60, 50, 40};

/** @def emg_chunk
 * @brief Bloque de n-CHUNK datos
 */
volatile float emg_chunk[CHUNK];

/** @def emg_filtrado
 * @brief Bloque de n-CHUNK datos filtrados
 */
volatile float emg_filtrado[CHUNK];

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
    xTaskNotifyGive(alert_task_handle);
}

/**  @def void EnviarDatos(float *datos)
 * @brief Función que envía los datos por UART
 * @param[in] datos float* que corresponde a los datos a enviar
 */
void EnviarDatos(float *datos)
{
    for (uint8_t indice = 0; indice < CHUNK; indice++) // Envia por UART bloques de datos de n-CHUNK datos
    {
        UartSendString(UART_PC, (char *)UartItoa(datos[indice], 10)); 
        UartSendString(UART_PC, "\r\n");
    }
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
    //Aca tengo que ver si va un filtro pasa alto
    LowPassFilter(senialRectificada, emg_salida, CHUNK);
}

/**  @def void ControlMovLEDS(uint8_t indice)
 * @brief Función que controla el movimiento de los LEDs
 * @param[in] indice uint8_t que corresponde al indice del arreglo de datos
 */
void ControlMovLEDS(uint8_t indice)
{
    // de ultima si no funca a la mierda--> Por el momento no funca

    float porcentaje = senialPrueba2[indice] / umbral;
    if (porcentaje > 1)
    {
        porcentaje = 1;
    }
    float LedsActivos = roundf(porcentaje * N_LEDS); // Casteo a int(?)
    NeoPixelAllColor(0);
    for (uint16_t pos_led = 0; pos_led < LedsActivos; pos_led++)
    {
        NeoPixelSetPixel(pos_led, NEOPIXEL_COLOR_BLUE);
    }
}

/* //Segun peña en este proyecto no hace falta
void CalibrarCeldaCarga(float pesoReal){
    HX711_tare(10);
    uint32_t lecturaPeso = HX711_read();//De ultima HX711_readAverage(10);
    float escala = lecturaPeso/pesoReal;
    HX711_setScale(escala);


}*/

/**  @def void MedirFuerza(void)
 * @brief Función que mide la fuerza aplicada
 */
void MedirFuerza(void)
{
    float fuerza = 0;

    fuerza = HX711_get_units(5);
    printf("Valor de  Fuerza: %f", fuerza);
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

        if (indice < 22)
        {
            NeoPixelAllOff(); // Apago todos para prenderlos rojo

            if (senialPrueba2[indice] > umbral) // Si el valor de la senial es mayor al umbral se activa la tira LED en rojo y el buzzer
            {

                // BuzzerPlayTone(NOTE_A6,300);
                NeoPixelAllColor(NEOPIXEL_COLOR_RED);                     
                // MedirFuerza();
                // printf("%d",senialPrueba2[indice]); //esto no me funcionaba no se por que
                // printf("\r\n");
            }
            else
            {
                ControlMovLEDS(indice); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                        // La simplificaria a esta funcion, si esta -50% verde, 50% a 90% amarillo,
                                        // +90% naranja   
            }
            indice++;
            //printf("%d", senialPrueba2[indice]);
            //printf("\r\n");
        }
        else if (indice == 22)
        {
            indice = 0;
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
            //AplicarFiltrado(emg_chunk, emg_filtrado, CHUNK);
            //si encuentra el umbral que mida la fuerza de la mano, emite alerta
            //EnviarDatosUART(emg_filtrado);//para ver los datos

            strcpy(msg, ""); // Limpia el contenido de la variable msg

			// Envía el chunk procesado por puerto serie
			for (uint8_t k = 0; k < CHUNK; k++) // Recorre emg_filtrado
			{
				sprintf(msg_chunk, "%.2f\r\n", emg_filtrado[k]); // Castea el valor emg_filtrado[k] en un string y lo almacena en msg_chunk
				strcat(msg,msg_chunk); // Concatena el string msg_chunk al string msg
			}
			printf(msg); // Muestra el string msg en la consola
            
            contador = 0; // Reinicia el contador
        }
    }
}
/*==================[external functions definition]==========================*/
void app_main(void){

    /*Variable declarations*/
    static neopixel_color_t color[N_LEDS];
    
    /*Inicializations*/
    LowPassInit(SAMPLE_FREQ, 30, ORDER_2); // Inicializo el filtro pasa bajo
    HiPassInit(SAMPLE_FREQ, 1, ORDER_2); // Inicializo el filtro pasa alto
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
        .period = T_SENIAL,
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
    HX711_tare(10); // Tara el 0 en el sensor
    HX711_setScale(2.436); // Calibrado con 100g
    printf("Calibracion Ralizada \r\n");

    /*Tasks*/
    xTaskCreate(EMGTask, "EMG", 4096, NULL, 5, &emg_task_handle);
    xTaskCreate(BuzzerLedTask, "Buzzer", 2048, NULL, 5, &alert_task_handle);

    /*Timers start*/
    TimerStart(timer_senial.timer);
    TimerStart(timer_alerta.timer);


    //!!!!!!!!!!!!!!!!!!!!!!!!!!! La fuerza no se mide contantemente





    NeoPixelAllOff();
    // BuzzerOn();

    // SCL: GPIO_7
    // SDA: GPIO_6
    
    // CalibrarCeldaCarga(200);

    NeoPixelAllColor(NEOPIXEL_COLOR_GREEN);
    // vTaskDelay(700 / portTICK_PERIOD_MS);
    // NeoPixelAllOff();



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

// Posible tono para el umbral alerta:d=8,o=5,b=150:c6,e6,g6
/*==================[end of file]============================================*/
