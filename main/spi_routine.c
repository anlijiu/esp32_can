#include "spi_routine.h"
#include <string.h>
#include "esp_system.h"
#include "esp_err.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"


#ifdef CONFIG_IDF_TARGET_ESP32
#define MY_SPI_HOST    HSPI_HOST

#define PIN_NUM_CS   15
#define PIN_NUM_CLK  14
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define MY_SPI_HOST    SPI2_HOST

#define PIN_NUM_MISO 37
#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK  36
#define PIN_NUM_CS   34
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define MY_SPI_HOST    SPI2_HOST

#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK  6
#define PIN_NUM_CS   10

#elif defined CONFIG_IDF_TARGET_ESP32S3
// https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html
#define MY_SPI_HOST    SPI2_HOST
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

#endif

#define MCP "SPI"

static spi_device_handle_t spi;

esp_err_t init_spi() {
	esp_err_t ret;

	spi_bus_config_t buscfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=4096
	};
	spi_device_interface_config_t devcfg={
		.clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
		.mode=0,                                //SPI mode 0 就是  spi 0,0
		.spics_io_num=PIN_NUM_CS,               //CS pin
		.queue_size=7,
		// .pre_cb=lcd_spi_pre_transfer_callback,
	};

		//Initialize the SPI bus
    ret = spi_bus_initialize(MY_SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    // ret = spi_bus_initialize(MY_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);
	//Attach device to the SPI bus
	ret = spi_bus_add_device(MY_SPI_HOST, &devcfg, &spi);
	gpio_set_level(PIN_NUM_CS, 0);
	ESP_ERROR_CHECK(ret);
	return ret;
}

void spi_cs_low(){
	// gpio_set_level(PIN_NUM_CS, 0);
}

void spi_cs_high(){
	// gpio_set_level(PIN_NUM_CS, 1);
}

esp_err_t write_register(uint8_t addr, uint8_t val) {
	ESP_LOGI(MCP, "write_register: addr:%02x, val:%02x", addr, val);
	esp_err_t ret;
	spi_transaction_t t;

	uint8_t data[3];
	data[0]=0x02;
	data[1]=addr;
	data[2]=val;

	memset(&t, 0, sizeof(t));
	t.length=24;
	t.tx_buffer=&data;
	ret = spi_device_transmit(spi, &t);
	return ret;
}

esp_err_t mod_register(uint8_t addr, uint8_t mask, uint8_t val) {
	ESP_LOGI(MCP, "mod_register: addr:%02x, mask:%02x, val: %02x\n", addr, mask, val);
	spi_transaction_t t;
	esp_err_t ret;

	uint8_t data[4];
	data[0]=0x05;
	data[1]=addr;
	data[2]=mask;
	data[3]=val;

	memset(&t, 0, sizeof(t));
	t.length=32;
	t.tx_buffer=&data;
	ret=spi_device_transmit(spi, &t);
	return ret;
}

//读寄存器
uint8_t read_reg(uint8_t addr) {
	spi_transaction_t r;

	uint8_t data[2];
	data[0] = 0x03;
	data[1] = addr;

	memset(&r, 0, sizeof(r));
	r.length = 8*4;
	r.tx_buffer = &data;
	r.flags = SPI_TRANS_USE_RXDATA;

	spi_device_polling_transmit(spi, &r);
	ESP_LOGI(MCP, "r: [%02x, %02x, %02x, %02x], addr:%02x",
		r.rx_data[0], r.rx_data[1], r.rx_data[2], r.rx_data[3], addr);
	return r.rx_data[2];
}

esp_err_t send_data(const uint8_t *data, int len) {
	ESP_LOGI(MCP, "send_data: len:%d\n", len);
	esp_err_t ret;
	spi_transaction_t t;
	if (len==0) return 0;
	memset(&t, 0, sizeof(t));
	t.length = len*8;
	t.tx_buffer = data;
	ret = spi_device_polling_transmit(spi, &t);  //Transmit!
	return ret;
}
