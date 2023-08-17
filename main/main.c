/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "blespp.h"
#include "can_proto.h"
#include "spi_routine.h"
#include "mcp2515.h"

#define MAIN_TAG "MAIN"

QueueHandle_t can_frame_queue = NULL;
SemaphoreHandle_t cdc_mtx;
QueueHandle_t can_irq_quee = NULL;

void app_main(void)
{

	can_frame_queue = xQueueCreate(10, sizeof(uint32_t));
	can_irq_quee = xQueueCreate(10, sizeof(uint32_t));
	cdc_mtx = xSemaphoreCreateMutex();

    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    init_blespp();
	ret = init_spi();
	if (ret) {
		ESP_LOGE(MAIN_TAG, "%s initialize spi failed\n", __func__);
		return;
	}

    init_can();

	bootstrap_mcp();

    return;
}
