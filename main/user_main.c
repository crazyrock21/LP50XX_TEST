/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "string.h"

#include "lp50xx.h"

#define	GPIO_INPUT_DETECT_PLUG		16
#define	GPIO_INPUT_KEY_DETECT		32
#define	GPIO_INPUT_VOL_UP_BTN		23
#define	GPIO_INPUT_VOL_DOWN_BTN		22

#define	GPIO_OUTPUT_LED_DRIVER_EN	17
#define	GPIO_OUTPUT_SUB_PWR_EN		18
#define	GPIO_OUTPUT_PWR_EN			19
#define	GPIO_OUTPUT_BOOST_EN		21

#define GPIO_INPUT_PIN_SEL  		((1ULL<<GPIO_INPUT_DETECT_PLUG) | (1ULL<<GPIO_INPUT_KEY_DETECT) | (1ULL<<GPIO_INPUT_VOL_UP_BTN) | (1ULL<<GPIO_INPUT_VOL_DOWN_BTN))

#define	GPIO_OUTPUT_PIN_SEL		((1ULL<<GPIO_OUTPUT_LED_DRIVER_EN) | (1ULL<<GPIO_OUTPUT_SUB_PWR_EN) | (1ULL<<GPIO_OUTPUT_PWR_EN) | (1ULL<<GPIO_OUTPUT_BOOST_EN)) 

#define I2C_MASTER_TX_BUF_DISABLE 	0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 	0                           /*!< I2C master doesn't need buffer */
#define	I2C_MASTER_NUM				0
#define	I2C_MASTER_SCL_IO			12
#define	I2C_MASTER_SDA_IO			14
#define	I2C_MASTER_FREQ_HZ			100000
#define WRITE_BIT 					I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT 					I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 				0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 				0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 					0x0                             /*!< I2C ack value */
#define NACK_VAL 					0x1                            /*!< I2C nack value */

#define	LP5012_SLA					0x14

typedef enum {
	LOW,
	HIGH,
} HIGH_LOW;

static const char *TAG = "MAIN";

void gpio_init (void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;		
    io_conf.pull_up_en = 1;		
    gpio_config(&io_conf);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t slave_address, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_address << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t slave_address, const uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_byte(i2c_port_t i2c_num, uint8_t slave_address, uint8_t reg_address, uint8_t *data) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_address << 1) | READ_BIT, ACK_CHECK_EN);

    i2c_master_read_byte(cmd, data, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_write_byte(i2c_port_t i2c_num, uint8_t slave_address, uint8_t reg_address, uint8_t data) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_write_bytes(i2c_port_t i2c_num, uint8_t slave_address, uint8_t reg_address, uint8_t *data, size_t size) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);

	for (uint8_t i = 0; i < size; i++) {
		i2c_master_write_byte(cmd, *(data + i), ACK_CHECK_EN);
	}

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/*
void power_on_led (void)
{
	i2c_write_bytes (I2C_MASTER_NUM, LP5012_SLA, REG_LED0_BRIGHTNESS, cnt, 4);

}
*/

void led_color_reset (void)
{
}

void led_dim_task (void *pvParameter)
{
	uint8_t i = 0;
	uint8_t cnt[4] = {0};

	while (1) {
		i2c_write_bytes (I2C_MASTER_NUM, LP5012_SLA, REG_LED0_BRIGHTNESS, cnt, 4);

		cnt[0]++;
		cnt[1]++;
		cnt[2]++;
		cnt[3]++;

		i++;

		if (i > 100) {
			/*
			i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_RESET, 0xff);
			i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_DEVICE_CONFIG0, (1 << CHIP_EN_BIT));
			i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED1, 0xff);
			*/
		}

		vTaskDelay (10 / portTICK_RATE_MS);
	}

    vTaskDelete(NULL);
}
void timer_task (void *pvParameter)
{
	uint8_t buf = 0;

	while (1) {
	
		if (gpio_get_level (GPIO_INPUT_DETECT_PLUG) == LOW) {
			printf ("detect plug\n");
		}
		if (gpio_get_level (GPIO_INPUT_KEY_DETECT) == LOW) {
			printf ("key detect\n");
		}
		if (gpio_get_level (GPIO_INPUT_VOL_UP_BTN) == LOW) {
			printf ("vol up\n");
		}
		if (gpio_get_level (GPIO_INPUT_VOL_DOWN_BTN) == LOW) {
			printf ("vol down\n");
		}

		i2c_read_byte(I2C_MASTER_NUM, LP5012_SLA, 0x00, &buf);
		printf ("%02x\n", buf);
		i2c_read_byte(I2C_MASTER_NUM, LP5012_SLA, 0x01, &buf);
		printf ("%02x\n", buf);
		i2c_read_byte(I2C_MASTER_NUM, LP5012_SLA, 0x0b, &buf);
		printf ("%02x\n", buf);

		vTaskDelay (1000 / portTICK_RATE_MS);
	}

    vTaskDelete(NULL);
}

void app_main(void)
{
    printf("LP5012 TEST\n");

	gpio_init ();
    ESP_ERROR_CHECK (i2c_master_init());
	gpio_set_level(GPIO_OUTPUT_LED_DRIVER_EN, HIGH);
	gpio_set_level(GPIO_OUTPUT_SUB_PWR_EN, HIGH);
	gpio_set_level(GPIO_OUTPUT_PWR_EN, HIGH);
	gpio_set_level(GPIO_OUTPUT_BOOST_EN, HIGH);

	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_DEVICE_CONFIG0, (1 << CHIP_EN_BIT));

	uint8_t data[4] = {0};
	i2c_write_bytes (I2C_MASTER_NUM, LP5012_SLA, REG_LED0_BRIGHTNESS, data, 4);

	/*
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED1, 0xff);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED2, 0xff);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED3, 0xff);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED4, 0xff);
	*/

	/*
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED1, 0xff);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED2, 0xff);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED3, 0xff);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED4, 0xff);
	*/


	while (1) {
		/*
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_RESET, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_DEVICE_CONFIG0, (1 << CHIP_EN_BIT));
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED1, 0xff);
		*/

		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED1, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED2, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED3, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED4, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, BLUE_COLOR_LED1, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, BLUE_COLOR_LED2, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, BLUE_COLOR_LED3, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, BLUE_COLOR_LED4, 0xff);

		for (uint8_t j = 0; j < 4; j++) {
			for (uint8_t i = 0; i < 255; i++) {
				i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED0_BRIGHTNESS + j, i);
				//vTaskDelay (10 / portTICK_RATE_MS);
				for (uint16_t k = 0; k < 10000; k++)
					continue;
			}
		}

		vTaskDelay (1000 / portTICK_RATE_MS);

		for (uint8_t j = 0; j < 4; j++) {
			for (uint8_t i = 0; i < 255; i++) {
				i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED3_BRIGHTNESS - j, 254 - i);
				//vTaskDelay (10 / portTICK_RATE_MS);
				for (uint16_t k = 0; k < 10000; k++)
					continue;
			}
		}
		vTaskDelay (1000 / portTICK_RATE_MS);


		/*
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_RESET, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_DEVICE_CONFIG0, (1 << CHIP_EN_BIT));
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED1, 0xff);
		*/

		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, BLUE_COLOR_LED1, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, BLUE_COLOR_LED2, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, BLUE_COLOR_LED3, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, BLUE_COLOR_LED4, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED1, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED2, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED3, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED4, 0xff);

		for (uint8_t j = 0; j < 10; j++) {
			memset (data, 100, 4);
			i2c_write_bytes (I2C_MASTER_NUM, LP5012_SLA, REG_LED0_BRIGHTNESS, data, 4);
			vTaskDelay (200 / portTICK_RATE_MS);
			memset (data, 0, 4);
			i2c_write_bytes (I2C_MASTER_NUM, LP5012_SLA, REG_LED0_BRIGHTNESS, data, 4);
			vTaskDelay (200 / portTICK_RATE_MS);
		}

		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED1, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED2, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED3, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, RED_COLOR_LED4, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED1, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED2, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED3, 0xff);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED4, 0xff);

		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED0_BRIGHTNESS, 100);
		vTaskDelay (500 / portTICK_RATE_MS);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED1_BRIGHTNESS, 100);
		vTaskDelay (500 / portTICK_RATE_MS);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED2_BRIGHTNESS, 100);
		vTaskDelay (500 / portTICK_RATE_MS);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED3_BRIGHTNESS, 100);
		vTaskDelay (500 / portTICK_RATE_MS);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED3_BRIGHTNESS, 0x00);
		vTaskDelay (500 / portTICK_RATE_MS);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED2_BRIGHTNESS, 0x0);
		vTaskDelay (500 / portTICK_RATE_MS);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED1_BRIGHTNESS, 0x0);
		vTaskDelay (500 / portTICK_RATE_MS);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED0_BRIGHTNESS, 0x0);
		vTaskDelay (500 / portTICK_RATE_MS);

		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED1, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED2, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED3, 0x00);
		i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, GREEN_COLOR_LED4, 0x00);
	}

	/*
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED0_BRIGHTNESS, 0xa0);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED1_BRIGHTNESS, 0xa0);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED2_BRIGHTNESS, 0xa0);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_LED3_BRIGHTNESS, 0xa0);

	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT0_COLOR, 0xff);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT1_COLOR, 0x00);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT2_COLOR, 0x00);

	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT3_COLOR, 0x00);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT4_COLOR, 0xff);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT5_COLOR, 0x00);

	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT6_COLOR, 0x00);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT7_COLOR, 0x00);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT8_COLOR, 0xff);

	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT9_COLOR, 0x10);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT10_COLOR, 0x20);
	i2c_write_byte(I2C_MASTER_NUM, LP5012_SLA, REG_OUT11_COLOR, 0x03);
	*/

	if (xTaskCreate (&led_dim_task, "led_dim_task", 1024 * 2, NULL, 4, NULL) != pdPASS) {
		ESP_LOGE (TAG, "create timer_mode task failed");
	}

	if (xTaskCreate (&timer_task, "timer_task", 1024 * 2, NULL, 4, NULL) != pdPASS) {
		ESP_LOGE (TAG, "create timer_mode task failed");
	}

}
