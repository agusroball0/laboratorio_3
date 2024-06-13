#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "../components/led_strip/src/led_strip_rmt_ws2812.c"

#define TX_PIN (43)
#define RX_PIN (44)
#define TEST_RTS (-1)
#define TEST_CTS (-1)

#define UART_PORT_NUM      (0) 
#define UART_BAUD_RATE     (115200)
#define TASK_STACK_SIZE    (2048)

#define UART_BUFFER_SIZE   (1024)

/* ----- VARIABLES GLOBALES ----- */

QueueHandle_t color_queue;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} color_t;

// Variables globales de color
uint32_t RED = 255;
uint32_t GREEN = 0;
uint32_t BLUE = 255;

// Puntero a la estructura led_strip_t
led_strip_t *strip;


/* ----- TASK A: Encender y apagar un LED RGB cada segundo ----- */

void encender_led(void){
    strip->set_pixel(strip, 0, RED, GREEN, BLUE);
    strip->refresh(strip, 100);
}

void apagar_led(void){
    strip->set_pixel(strip, 0, 0, 0, 0);
    strip->refresh(strip, 100);
}

void blink_led(void *pvParameters) {
    while (true) {
        encender_led();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        apagar_led();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}

/* ----- TASK B: Recibir datos por UART y enviar a una QUEUE ----- */

void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TX_PIN, RX_PIN, TEST_RTS, TEST_CTS);
    uart_driver_install(UART_PORT_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);
}

void timer_callback(TimerHandle_t xTimer) {
    color_t* color = (color_t*) pvTimerGetTimerID(xTimer);

    // Enviar el color a la queue 
     if (xQueueSend(color_queue, color, portMAX_DELAY) != pdPASS) {
                printf("Error al enviar datos a la queue\n");
            }

    printf("Color (%d,%d,%d) enviado a la queue\n", color->red, color->green, color->blue);
    free(color);  // Liberar la memoria asignada al color del timer
    
    // Eliminar el timer para evitar fugas de memoria
    xTimerDelete(xTimer, 0);
}

void uart_receive_task(void *pvParameters) {
    // Configurar un buffer temporal para los datos entrantes
    uint8_t *data = (uint8_t *) malloc(UART_BUFFER_SIZE);
    if (data == NULL) {
        printf("Failed to allocate memory for UART data buffer\n");
        vTaskDelete(NULL);
    }

    while (true) {
        int len = uart_read_bytes(UART_PORT_NUM, data, (UART_BUFFER_SIZE - 1), 20 / portTICK_PERIOD_MS);
        uart_write_bytes(UART_PORT_NUM, (const char *) data, len);
        if (len > 0) {
            data[len] = '\0'; // Asegurar que los datos formen una cadena válida
            printf("Recv str: %s\n", (char *) data);

            // Parsear el string recibido
            char *token = strtok((char *)data, ", ");
            if (token == NULL) {
                printf("Dato no válido\n");
                continue;
            }
            char color_char = token[0];  // Primer caracter es el color

            token = strtok(NULL, ", ");
            if (token == NULL) {
                printf("Dato no válido\n");
                continue;
            }
            
            int delay_seconds = atoi(token); // Convertir la segunda parte en entero para el delay
            
            // Determinar el color según el caracter
            color_t *color = malloc(sizeof(color_t));  // Crear memoria para el color del timer
            if (color == NULL) {
                printf("Error al asignar memoria para el color del timer\n");
                continue;
            }

            switch (color_char) {
                case 'r':
                    *color = (color_t){.red = 255, .green = 0, .blue = 0};
                    break;
                case 'g':
                    *color = (color_t){.red = 0, .green = 255, .blue = 0};
                    break;
                case 'b':
                    *color = (color_t){.red = 0, .green = 0, .blue = 255};
                    break;
                default:
                    printf("Color no reconocido\n");
                    free(color);
                    continue; // Si el color no es reconocido, ignorar este comando
            }

            // Inicializar timer
            TimerHandle_t timer = xTimerCreate("Timer", pdMS_TO_TICKS(delay_seconds * 1000), pdFALSE, (void*)color, timer_callback);
            if (timer == NULL) {
                printf("Error al crear el timer\n");
                free(color);
            } else {
                // Enviar el color a la queue después del delay especificado
                xTimerStart(timer, 0);
            }
        }
    }
}

/* ----- TASK C: Actualizar el color de la tira de LED ----- */
void update_color_task(void *pvParameters) {
    color_t received_color;
    while (true) {
        if (xQueueReceive(color_queue, &received_color, portMAX_DELAY) == pdTRUE) {
            // Actualiza las variables globales con los nuevos valores de color
            RED = received_color.red;
            GREEN = received_color.green;
            BLUE = received_color.blue;
        }
    }
}

void app_main(void)
{
    // Inicializar el LED
    if (led_rgb_init(&strip) != ESP_OK) {
        printf("Error al inicializar la tira de LED\n"); // Si hay error se imprime por pantalla
        return;
        }

    // Inicializar UART
    init_uart();

    // Inicializar QUEUE
    color_queue = xQueueCreate(10, sizeof(color_t));
    if (color_queue == NULL) {
        printf("Error al crear la queue\n");
        return;
    }

    // Crear TASKS
    xTaskCreate(blink_led, "TaskA", TASK_STACK_SIZE, (void*)strip, 1, NULL);
    xTaskCreate(uart_receive_task, "TaskB", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(update_color_task, "TaskC", TASK_STACK_SIZE, NULL, 10, NULL);
  
}