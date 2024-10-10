/* SOFT UART Example for very slow communication FOR ESP32

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
#include "driver/uart.h"

#define ON 0X01
#define OFF 0X00

//UART COM1 COMMUNICATION 
#define UART_PORT UART_NUM_0      
#define RX_BAUD_RATE 115200
#define RX_BUF_SIZE  1024

//SOFT UART COMMUNICATION - ONLY TX 
#define TX_BAUD_RATE 10       //from 1 up to 100 bps
#define TX_PIN GPIO_NUM_15    // CHOOSE THE PIN
#define PARITY_ON OFF         //ODD PARITY ON|OFF   
#define LEVEL_HIGH OFF        // ON|OFF SERIAL SIGNAL INVERSION - HERE YOU CAN INVERT THE SERIAL LOGIC 1 TO 0
#define LEVEL_LOW !LEVEL_HIGH 


#define STRING_SIZE 120
typedef struct data_buff_ {
  char To_Send[STRING_SIZE];
  int size;
} data_buff;


#define QUEUE_SIZE 120  // Tamanho máximo da fila
typedef struct {
    data_buff buffer[QUEUE_SIZE]; // Array de elementos do tipo data_buff
    int head;                      // Índice do primeiro elemento da fila
    int tail;                      // Índice do próximo elemento disponível
    int count;                     // Número de elementos na fila
} fifo_queue;


void uart_soft_tx_chars(data_buff *payload);
QueueHandle_t spp_uart_queue = NULL;
fifo_queue q;
fifo_queue q_tx;
data_buff item;

//QUEUE FUNCTIONS
void init_queue(fifo_queue *q)
{
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

int enqueue(fifo_queue *q, data_buff item)
{
    if (q->count == QUEUE_SIZE)
    {
        // Fila cheia
        return -1;
    }

    q->buffer[q->tail] = item;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count++;

    return 0; // Sucesso
}

int dequeue(fifo_queue *q, data_buff *item)
{
    if (q->count == 0)
    {
        // Fila vazia
        return -1;
    }

    *item = q->buffer[q->head];
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->count--;

    return 0; // Sucesso
}

int is_empty(fifo_queue *q)
{
    return (q->count == 0);
}

int is_full(fifo_queue *q)
{
    return (q->count == QUEUE_SIZE);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = RX_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver, and get the queue.
    uart_driver_install(UART_PORT, 4096, 8192, 10, &spp_uart_queue, 0);
    // Set UART parameters
    uart_param_config(UART_PORT, &uart_config);
    // Set UART pins
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //xTaskCreate(uart_task, "uTask", 2048, (void *)UART_NUM_0, 8, NULL);
}

void uart_task_rx(void *pvParameters)
{
    uart_event_t event;
    data_buff item;

    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            switch (event.type)
            {
            // Event of UART receving data
            case UART_DATA:
                if ((event.size))
                {
                    uint8_t *temp = NULL;

                    temp = (uint8_t *)malloc(sizeof(uint8_t) * event.size);
                    if (temp == NULL)break;
                    memset(temp, 0x0, event.size);
                    uart_read_bytes(UART_PORT, temp, event.size, portMAX_DELAY);
                    item.size=event.size;
                    char *char_ptr = (char *)temp;
                    strncpy(item.To_Send, char_ptr, event.size);
                    enqueue(&q_tx, item);                  
                    enqueue(&q, item);
                    
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void uart_soft_tx_chars(data_buff *payload){
 ///TX at 10baud
  uint8_t data_tx=0;
  uint8_t i=0,j=0;
    
    gpio_set_level(TX_PIN, LEVEL_HIGH);
    while (j<payload->size) {
        gpio_set_level(TX_PIN, LEVEL_HIGH);
        vTaskDelay((5*1000/TX_BAUD_RATE) / portTICK_PERIOD_MS);
        gpio_set_level(TX_PIN, LEVEL_LOW);
        vTaskDelay((1000/TX_BAUD_RATE) / portTICK_PERIOD_MS); //start bit
        data_tx=payload->To_Send[j];

        uint8_t parity_bit = LEVEL_LOW;     

        uint8_t count_ones = 0;     
        for (uint8_t i = 0; i < 8; i++) {
            if (payload->To_Send[j] & (1 << i)) {
                count_ones++;
            }
        }

        // parity calculus
        if (count_ones % 2 != 0) {
            parity_bit = LEVEL_HIGH;
        }

        i=0;
        while (i<=7){
               uint8_t bit;
               if(LEVEL_HIGH)bit=data_tx & 1;
               if(LEVEL_LOW)bit=!(data_tx & 1);
               gpio_set_level(TX_PIN, bit);
               data_tx=data_tx >> 1;
               i++; 
               vTaskDelay((1000/TX_BAUD_RATE) / portTICK_PERIOD_MS);
        }
        if (PARITY_ON){
          gpio_set_level(TX_PIN, !parity_bit);
          vTaskDelay((1000/TX_BAUD_RATE) / portTICK_PERIOD_MS); //ODD parity bit
        }
        gpio_set_level(TX_PIN, LEVEL_HIGH);
        vTaskDelay((1000/TX_BAUD_RATE) / portTICK_PERIOD_MS); //stop bit

        j++; 

    }

}

void soft_uart_task_tx(void *pvParameters){

  data_buff payload;
    
  esp_rom_gpio_pad_select_gpio(TX_PIN);
  gpio_set_direction(TX_PIN, GPIO_MODE_OUTPUT);
  gpio_pullup_en(TX_PIN);
  gpio_pulldown_dis(TX_PIN);
  gpio_set_level(TX_PIN, LEVEL_HIGH);


  while (1) {
    while (dequeue(&q, &payload) == 0){
             uart_soft_tx_chars(&payload);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);            
  }
}

void uart_task_tx(void *pvParameters){
  data_buff payload;
  while (1) {
    while (dequeue(&q_tx, &payload) == 0){
             //echo on the Terminal
             uart_write_bytes(UART_PORT,payload.To_Send,payload.size);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);            
  }
}

void app_main(void)
{
   spp_uart_init();
   xTaskCreate(&uart_task_rx, "uart_task_rx",4096, NULL, 5, NULL);
   xTaskCreate(&uart_task_tx, "uart_task_tx",4096, NULL, 5, NULL);  
   xTaskCreate(&soft_uart_task_tx, "soft_uart_task_tx",4096, NULL, 5, NULL); 
   vTaskDelay(1000 / portTICK_PERIOD_MS); //stop bit
   printf("\nSOFT UART EXAMPLE\n");
   printf("\nSetup the Logic Analyser connected on PIN:%d @ TX BAUD:%d\n",TX_PIN,TX_BAUD_RATE);
   printf("and type any key...\n");
   while(1){
           vTaskDelay(100 / portTICK_PERIOD_MS);            
   }
}
