
/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "uart_programmer.h"
#include "eeprom_controller.hpp"
static const char *TAG = "uart_events";

#include "babel.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;
#define pattern_chr '+'
#define num_symbols 3 
static uint8_t s_consecutive_symbols = 0;
typedef enum program_storage {
    non_active = 0,
    active = 1,
} pstate;
static pstate cur_state;
static uint8_t *program_buffer = NULL;
static bool program_ready = false;
static int current_index = 0;
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    dtmp[event.size] = 0;
#define SFRAME 0x02
#define EFRAME 0x3
#define MSG_SIZE_BUF 512 
                    for(int i = 0; i < event.size; ++i)
                    {
                        switch(dtmp[i]) 
                        {
                            case SFRAME:
                                ESP_LOGI(TAG, "start of frame");
                                //begin collecting new message 
                                if(program_buffer != NULL)
                                {
                                    free(program_buffer);
                                }
                                program_buffer = (uint8_t*) malloc(MSG_SIZE_BUF);
                                current_index = 0;
                                break;
                            case EFRAME:
                                ESP_LOGI(TAG, "end of frame");
                                //
                                //parse received message
                                //
                                if(current_index%2) 
                                {
                                    ESP_LOGE(TAG, "babel message odd size(%d)", current_index);
                                    free(program_buffer);
                                    program_buffer = NULL;
                                    continue;
                                }

                                for(int j = 0; j < current_index; j+=2)
                                {
                                    program_buffer[j/2] = 
                                        strtol((char[]){program_buffer[j],
                                                         program_buffer[j+1], 0},
                                                            NULL, 16);
                                }

                                ESP_LOGI(TAG, "transformed msg:");
                                esp_log_buffer_hex(TAG, program_buffer, current_index/2);

                                pb_istream_t stream = pb_istream_from_buffer(
                                        program_buffer,
                                        current_index/2);

                                /* Now we are ready to decode the message. */
                                programTransfer msg = programTransfer_init_zero;

                                bool status = pb_decode(&stream, programTransfer_fields, &msg);
                                QueueHandle_t target_queue = get_programmer_queue();
                                if(target_queue != NULL)
                                {
                                    xQueueSendToBack(
                                            target_queue,
                                            &msg,
                                            5000/ portTICK_RATE_MS
                                            );
                                }

                                break;
                            default:
                                if(program_buffer == NULL)
                                {
                                    //error case
                                    continue;
                                }
                                program_buffer[current_index] = dtmp[i]; 
                                current_index++;
                                break;
                        }
                        
                    }
                    ESP_LOGI(TAG, "[DATA EVT]: %s", dtmp);
                      
                    //uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void get_program(char **program, int *length)
{
    return NULL;    
}
void uart_programmer()
{

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 3096, NULL, 12, NULL);
}
