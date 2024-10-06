/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define TX_PIN GPIO_NUM_15
#define Parity OFF


#define ON 1
#define OFF 0

void app_main(void)
{
   ///TX at 10baud
  
    #define QUANT 10  // Número de caracteres no texto "###;flavio;ercolano;123456;"

    uint8_t data[QUANT] = {
        0x63, 0x61, 0x6C, 0x65, 0x6E, 0x64, 0x61, 0x72, 0x3B, 0x0D
          };


    uint8_t data_tx=0;
    uint8_t i=0,j=0;

    esp_rom_gpio_pad_select_gpio(TX_PIN);
    gpio_set_direction(TX_PIN, GPIO_MODE_OUTPUT);
    gpio_pullup_en(TX_PIN);
    gpio_pulldown_dis(TX_PIN);

  while(1){
    gpio_set_level(TX_PIN, ON);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    while (j<QUANT) {
        gpio_set_level(TX_PIN, ON);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(TX_PIN, OFF);
        vTaskDelay(100 / portTICK_PERIOD_MS); //start bit
        data_tx=data[j];

        uint8_t parity_bit = 0;     // Inicialmente 0 (paridade par)

        uint8_t count_ones = 0;     // Contador de bits 1
        for (uint8_t i = 0; i < 8; i++) {
            if (data[j] & (1 << i)) {
                count_ones++;
            }
        }

        // Se o número de bits 1 for ímpar, o bit de paridade é 1
        if (count_ones % 2 != 0) {
            parity_bit = 1;
        }

        i=0;
        while (i<=7){
               uint8_t bit=data_tx & 0x01;
               gpio_set_level(TX_PIN, bit);
               data_tx=data_tx >> 1;
               i++; 
               vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        if (Parity){
          gpio_set_level(TX_PIN, !parity_bit);
          vTaskDelay(100 / portTICK_PERIOD_MS); //ODD parity bit
        }
        gpio_set_level(TX_PIN, ON);
        vTaskDelay(100 / portTICK_PERIOD_MS); //stop bit

        j++; 

    }
    j=0;
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}
