void encender_led(led_strip_t *strip, uint32_t index, char color);
void encender_led_custom(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);
void apagar_led(led_strip_t *strip, uint32_t index);
void color_cycle(led_strip_t *strip, int speed_ms);
