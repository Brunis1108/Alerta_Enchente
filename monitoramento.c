#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>
#include "pico/bootrom.h"
#include "queue.h"
#include "ws2812.pio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

/* ================= DEFINIÇÕES DE HARDWARE ================= */
// Display OLED (I2C)
#define I2C_PORT i2c1
#define PIN_I2C_SDA 14
#define PIN_I2C_SCL 15
#define OLED_ADDRESS 0x3C

// Joystick (ADC)
#define JOYSTICK_X 26  // ADC0 - Eixo X (GPIO 26)
#define JOYSTICK_Y 27  // ADC1 - Eixo Y (GPIO 27)

// LEDs
#define LED_GREEN 11
#define LED_RED 13
#define WS2812_PIN 7    // Matriz de LEDs
#define NUM_LEDS 25     // 5x5 matrix

// Buzzer
#define BUZZER_PIN 10
#define REST 0

// Botão para modo BOOTSEL
#define BUTTON_B 6

/* ================= VARIÁVEIS GLOBAIS ================= */
PIO pio = pio0;        // Controlador PIO para LEDs WS2812
int sm = 0;            // Máquina de estado PIO
ssd1306_t display;     // Objeto do display OLED

// Filas para comunicação entre tarefas
QueueHandle_t displayQueue;
QueueHandle_t ledQueue;
QueueHandle_t buzzerQueue;

/* ================= ESTRUTURAS DE DADOS ================= */
typedef struct {
    uint8_t x;  // Nível da água (0-100)
    uint8_t y;  // Volume de chuva (0-100)
} JoystickData;

/* ================= PROTÓTIPOS DE FUNÇÕES ================= */
void display_init();
void led_init(int led_pin);
void acender_leds(uint32_t r, uint32_t g, uint32_t b);
void buzzer_init();
void buzzer_play_note(int freq, int duration_ms);
void gpio_irq_handler(uint gpio, uint32_t events);
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);
void put_pixel(uint32_t pixel_grb);

/* ================= TAREFAS DO FreeRTOS ================= */

// Tarefa: Leitura do joystick
void vJoystickTask(void *params) {
    // Inicializa ADCs
    adc_init();
    adc_gpio_init(JOYSTICK_X);
    adc_gpio_init(JOYSTICK_Y);

    JoystickData dados;

    while (true) {
        // Lê eixo X (ADC0)
        adc_select_input(0);
        dados.x = (adc_read() * 100) / 4095;

        // Lê eixo Y (ADC1)
        adc_select_input(1);
        dados.y = (adc_read() * 100) / 4095;

        // Debug no terminal
        printf("[Joystick] Água = %d%%, Chuva = %d%%\n", dados.x, dados.y);

        // Envia dados para as filas
        xQueueSend(displayQueue, &dados, portMAX_DELAY);
        xQueueSend(ledQueue, &dados, portMAX_DELAY);
        xQueueSend(buzzerQueue, &dados, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(200));  // 5Hz
    }
}

// Tarefa: Atualização do display OLED
void vDisplayTask(void *params) {
    ssd1306_fill(&display, false);  // Limpa display
    
    JoystickData dados;
    char status[20];

    while (true) {
        if (xQueueReceive(displayQueue, &dados, portMAX_DELAY)) {
            // Limpa display
            ssd1306_fill(&display, false);

            // Exibe nível da água
            ssd1306_draw_string(&display, "Agua: ", 2, 2);
            ssd1306_draw_char(&display, '0' + (dados.x / 10), 10, 20);
            ssd1306_draw_char(&display, '0' + (dados.x % 10), 18, 20);
            sprintf(status, "%s", (dados.x >= 70) ? "ALERTA" : "Normal");
            ssd1306_draw_string(&display, status, 7, 40);

            // Divisor
            ssd1306_line(&display, 64, 0, 64, 128, true);

            // Exibe volume de chuva
            ssd1306_draw_string(&display, "Chuva: ", 70, 2);
            ssd1306_draw_char(&display, '0' + (dados.y / 10), 84, 20);
            ssd1306_draw_char(&display, '0' + (dados.y % 10), 92, 20);
            sprintf(status, "%s", (dados.y >= 80) ? "ALERTA" : "Normal");
            ssd1306_draw_string(&display, status, 70, 40);

            // Atualiza display
            ssd1306_send_data(&display);
        }
    }
}

// Tarefa: Controle dos LEDs
void vLedsTask(void *params) {
    JoystickData dados;

    while (true) {
        if (xQueueReceive(ledQueue, &dados, portMAX_DELAY)) {
            // Modo Alerta (água alta ou chuva forte)
            if (dados.x >= 70 || dados.y >= 80) {
                // Alerta de água alta (vermelho)
                if (dados.x >= 70) {
                    acender_leds(10, 0, 0);  // Vermelho
                    gpio_put(LED_RED, true);
                }
                
                // Alerta de chuva forte (laranja)
                if (dados.y >= 80) {
                    acender_leds(15, 5, 0);  // Laranja
                    gpio_put(LED_RED, true);
                    gpio_put(LED_GREEN, true);
                }
                
                // Piscar LEDs
                vTaskDelay(pdMS_TO_TICKS(150));
                gpio_put(LED_RED, false);
                gpio_put(LED_GREEN, false);
                acender_leds(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(150));
            } 
            // Modo Normal
            else {
                gpio_put(LED_RED, false);
                gpio_put(LED_GREEN, false);
                acender_leds(0, 0, 0);
            }
        }
    }
}

// Tarefa: Controle do buzzer
void vBuzzerTask(void *params) {
    JoystickData dados;

    while (true) {
        if (xQueueReceive(buzzerQueue, &dados, portMAX_DELAY)) {
            // Modo Alerta
            if (dados.x >= 70 || dados.y >= 80) {
                // Tom para água alta (grave)
                if (dados.x >= 70) {
                    buzzer_play_note(200, 100);
                }
                // Tom para chuva forte (agudo)
                if (dados.y >= 80) {
                    buzzer_play_note(400, 200);
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            } 
            // Modo Normal (silêncio)
            else {
                buzzer_play_note(REST, 0);
            }
        }
    }
}

/* ================= FUNÇÕES DE INICIALIZAÇÃO ================= */

// Inicializa display OLED
void display_init() {
    i2c_init(I2C_PORT, 400 * 1000);  // I2C a 400kHz
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    ssd1306_init(&display, 128, 64, false, OLED_ADDRESS, I2C_PORT);
    ssd1306_config(&display);
    ssd1306_send_data(&display);
}

// Inicializa LED simples
void led_init(int led_pin) {
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    gpio_put(led_pin, false);
}

// Inicializa buzzer
void buzzer_init() {
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
}

/* ================= FUNÇÕES PARA MATRIZ DE LEDs ================= */

// Envia cor para um LED da matriz
void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

// Converte RGB para formato 32-bit
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
}

// Controla a matriz de LEDs
void acender_leds(uint32_t r, uint32_t g, uint32_t b) {
    for (int i = 0; i < NUM_LEDS; ++i) {
        // Desenha um triangulo na matriz de led
        if ((i >= 5 && i <= 9) || (i >= 11 && i <= 13) || (i == 17)) {
            put_pixel(urgb_u32(r, g, b));
        } else {
            put_pixel(urgb_u32(0, 0, 0));  // Apaga outros LEDs
        }
    }
}

/* ================= FUNÇÃO DO BUZZER ================= */

void buzzer_play_note(int freq, int duration_ms) {
    if (freq == REST) {
        gpio_put(BUZZER_PIN, 0);
        sleep_ms(duration_ms);
        return;
    }

    // Calcula período e ciclos para a nota
    uint32_t period_us = 1000000 / freq;
    uint32_t cycles = (freq * duration_ms) / 1000;

    // Gera onda quadrada
    for (uint32_t i = 0; i < cycles; i++) {
        gpio_put(BUZZER_PIN, 1);
        sleep_us(period_us / 2);
        gpio_put(BUZZER_PIN, 0);
        sleep_us(period_us / 2);
    }
}

/* ================= INTERRUPÇÃO PARA BOOTSEL ================= */

void gpio_irq_handler(uint gpio, uint32_t events) {
    reset_usb_boot(0, 0);  // Entra no modo BOOTSEL
}

/* ================= FUNÇÃO PRINCIPAL ================= */

int main() {
    stdio_init_all();
    sleep_ms(1000);  // Espera inicialização USB

    // Configura botão para modo BOOTSEL
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Inicializa matriz de LEDs
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false);

    // Inicializa componentes
    led_init(LED_RED);
    led_init(LED_GREEN);
    buzzer_init();
    display_init();

    // Cria filas para comunicação entre tarefas
    displayQueue = xQueueCreate(10, sizeof(JoystickData));
    ledQueue = xQueueCreate(10, sizeof(JoystickData));
    buzzerQueue = xQueueCreate(10, sizeof(JoystickData));

    // Cria tarefas
    xTaskCreate(vJoystickTask, "Joystick", 256, NULL, 1, NULL);
    xTaskCreate(vLedsTask, "LEDs", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer", 256, NULL, 1, NULL);

    // Inicia escalonador
    vTaskStartScheduler();

    // Nunca deve chegar aqui
    panic_unsupported();
}