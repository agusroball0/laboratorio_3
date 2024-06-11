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

// Variable global para la queue
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

// Crear un puntero a la estructura led_strip_t
led_strip_t *strip;

void encender_led(void){
    strip->set_pixel(strip, 0, RED, GREEN, BLUE);
    strip->refresh(strip, 100);
    // printf("LED encendido\n");

}

void apagar_led(void){
    strip->set_pixel(strip, 0, 0, 0, 0);
    strip->refresh(strip, 100);
    // printf("LED apagado\n");

}

void blink_led(void *pvParameters) {
    while (true) {
        encender_led();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        apagar_led();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}

void uart_receive_task(void *pvParameters) {
// Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(UART_BUFFER_SIZE);
    while (true) {
        int len = uart_read_bytes(UART_PORT_NUM, data, (UART_BUFFER_SIZE - 1), 20 / portTICK_PERIOD_MS);
        uart_write_bytes(UART_PORT_NUM, (const char *) data, len);
        if (len) {
            data[len] = '\0';
            printf("Recv str: %s\n", (char *) data);

            // Parsear el string recibido
            char *token = strtok((char *)data, ", ");
            char color_char = token[0];  // Primer caracter es el color
            token = strtok(NULL, ", ");
            int delay_seconds = atoi(token); // Convertir la segunda parte en entero para el delay

             // Determinar el color según el caracter
            color_t color;
            switch (color_char) {
                case 'r':
                    color = (color_t){.red = 255, .green = 0, .blue = 0};
                    break;
                case 'g':
                    color = (color_t){.red = 0, .green = 255, .blue = 0};
                    break;
                case 'b':
                    color = (color_t){.red = 0, .green = 0, .blue = 255};
                    break;
                default:
                    printf("Color no reconocido\n");
                    continue; // Si el color no es reconocido, ignorar este comando
            }

            // Enviar el color a la queue después del delay especificado
            vTaskDelay(pdMS_TO_TICKS(delay_seconds * 1000));
            if (xQueueSend(color_queue, &color, portMAX_DELAY) != pdPASS) {
                printf("Error al enviar datos a la queue\n");
            }

            // color_t color = {.red = data[0], .green = data[1], .blue = data[2]};
            // if (xQueueSend(color_queue, &color, portMAX_DELAY) != pdPASS) {
            //     printf("Error al enviar datos a la queue\n");
            // }
        }
    }
}

void timer_callback(TimerHandle_t xTimer) {
    color_t color;
    if (xQueueReceive(color_queue, &color, 0) == pdTRUE) {
        printf("Color recibido - Rojo: %d, Verde: %d, Azul: %d\n", color.red, color.green, color.blue);
    }
}

void app_main(void)
{
    
  // Inicializar el LED
    if (led_rgb_init(&strip) != ESP_OK) {
        printf("Error al inicializar la tira de LED\n"); // Si hay error se imprime por pantalla
        }

  // Inicializar UART
    init_uart();
    xTaskCreate(uart_receive_task, "uart_receive_task", 2048, NULL, 10, NULL);

  // Inicializar QUEUE
    color_queue = xQueueCreate(10, sizeof(color_t));
    if (color_queue == NULL) {
        printf("Error al crear la queue\n");
    }
  
  // Inicializar timers
    TimerHandle_t timer = xTimerCreate("Timer", pdMS_TO_TICKS(6000), pdTRUE, (void*)0, timer_callback);
    if (timer == NULL) {
        printf("Error al crear el timer\n");
    } else {
        xTimerStart(timer, 0);
    }


  // Crear tarea de control de LED
    xTaskCreate(blink_led, "Tarea1", 2048, (void*)strip, 1, NULL);
  
}
