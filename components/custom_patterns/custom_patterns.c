#include "../../led_control.h"
#include "custom_patterns.h"


void rapid_color_change(led_strip_t *strip, int speed_ms) {
    const char colors[] = {'R', 'B', 'G', 'Y', 'V', 'C', 'W'};  
    const int num_colors = sizeof(colors) / sizeof(colors[0]);
    
    for (int i = 0; i < num_colors; i++) {
        encender_led(strip, 0, colors[i]);
        delay_ms(speed_ms);  // Rapid change speed
        apagar_led(strip, 0);
        delay_ms(speed_ms);
    }
}


void slow_fade_to_black(led_strip_t *strip, int duration_ms) {
    const uint32_t colors[][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}};  // RGB values for Red, Green, Blue, Yellow
    const int num_colors = sizeof(colors) / sizeof(colors[0]);  // Correct way to find the number of colors
    
    for (int i = 0; i < num_colors; i++) {
        for (int j = 255; j >= 0; j--) {
            uint32_t r = (colors[i][0] * j) / 255;
            uint32_t g = (colors[i][1] * j) / 255;
            uint32_t b = (colors[i][2] * j) / 255;
            encender_led_custom(strip, 0, r, g, b);
            delay_ms(duration_ms / 255);  // Duration for fade
        }
        apagar_led(strip, 0);
        delay_ms(100);  // Brief pause before next color
    }
}

void slow_fade_to_color(led_strip_t *strip, int duration_ms){
    const uint32_t colors[][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}};  // RGB values for Red, Green, Blue, Yellow 
    const int num_colors = sizeof(colors) / sizeof(colors[0]);  // Correct way to find the number of colors

    for (int i = 0; i < num_colors; i++) {
        for (int j = 0; j < 255; j++) {
            uint32_t r = (colors[i][0] * j) / 255;
            uint32_t g = (colors[i][1] * j) / 255;
            uint32_t b = (colors[i][2] * j) / 255;
            encender_led_custom(strip, 0, r, g, b);
            delay_ms(duration_ms / 255);  // Duration for fade
        }
        apagar_led(strip, 0);
        delay_ms(100);  // Brief pause before next color
    }
    
}

void smooth_color_transition(led_strip_t *strip, int duration_ms) {
    const uint32_t colors[][3] = {{0, 0, 0},{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}, {0, 0, 0}};  // RGB values for Red, Green, Blue, Yellow, Black
    const int num_colors = sizeof(colors) / sizeof(colors[0]);
    const int steps = 255;  // Number of steps in the transition between colors

    for (int i = 0; i < num_colors - 1; i++) {
        uint32_t start_r = colors[i][0];
        uint32_t start_g = colors[i][1];
        uint32_t start_b = colors[i][2];
        uint32_t end_r = colors[i + 1][0];
        uint32_t end_g = colors[i + 1][1];
        uint32_t end_b = colors[i + 1][2];

        for (int j = 0; j <= steps; j++) {
            uint32_t r = start_r + (end_r - start_r) * j / steps;
            uint32_t g = start_g + (end_g - start_g) * j / steps;
            uint32_t b = start_b + (end_b - start_b) * j / steps;
            encender_led_custom(strip, 0, r, g, b);
            delay_ms(duration_ms / steps);  // Duration for each step of fade
        }
        // delay_ms(500);  // Brief pause before next color
    }
}
