#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "../components/led_strip/src/led_strip_rmt_ws2812.c"
// #include "../components/blocking_delay/blocking_delay.c"
// #include "../components/led_control/include/led_control.c"

// Variables globales de color
uint32_t RED = 255;
uint32_t GREEN = 0;
uint32_t BLUE = 255;

// Crear un puntero a la estructura led_strip_t
    led_strip_t *strip;

void encender_led(void){
    strip->set_pixel(strip, 0, RED, GREEN, BLUE);
    strip->refresh(strip, 100);
    printf("LED encendido\n");

}

void apagar_led(void){
    strip->set_pixel(strip, 0, 0, 0, 0);
    strip->refresh(strip, 100);
    printf("LED apagado\n");

}

void blink_led(void *pvParameters) {
    while (true) {
        encender_led();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        apagar_led();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}


void app_main(void)
{
    
    // Inicializar el LED
    if (led_rgb_init(&strip) != ESP_OK) {
        printf("Error al inicializar la tira de LED\n"); // Si hay error se imprime por pantalla
        }

  // Inicializar UART

  // Inicializar QUEUE
  
  // Inicializar timers

  // Crear tarea de control de LED
    xTaskCreate(blink_led, "Tarea1", 2048, (void*)strip, 1, NULL);
  
}
