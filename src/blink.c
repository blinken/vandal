/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include <unistd.h>

#define BLINK_GPIO 13


void blinky(void *pvParameter)
{

    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}



#define I2C_SCL_GPIO				16				//GPIO pin
#define I2C_SDA_GPIO				17				//GPIO pin
#define I2C_FREQ_HZ					100000			//!< I2C master clock frequency
#define I2C_ADDR					0x3e
#define I2C_WRITE_BIT				0
#define I2C_READ_BIT				1
#define I2C_ACK_CHECK_EN    true

bool i2c_write(uint8_t reg, uint8_t value, char *description) {

  uint8_t write_addr = (I2C_ADDR<<1) | I2C_WRITE_BIT;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, value, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	printf("Set %20s (0x%02x) = 0x%02x: ", description, reg, value);
	esp_err_t i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, (1000 / portTICK_RATE_MS));	//"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
  i2c_cmd_link_delete(cmd);

  if (i2c_ret == ESP_OK) {
    printf("succeeded.\n");
    return true;
  } else {
    printf("failed.\n");
    return false;
  }
}



void blinken_i2c_test() {
    //----- CREATE THE I2C PORT -----
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = 17;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = 16;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_FREQ_HZ;		//I2C frequency is the clock speed for a complete high low clock sequence
	i2c_param_config(I2C_NUM_1, &conf);
	i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0);

	esp_err_t i2c_ret = ESP_OK;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	vTaskDelay(5000 / portTICK_RATE_MS);


	//----- WRITE BYTES -----
  // regReset
	//i2c_master_start(cmd);
	//i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	//i2c_master_write_byte(cmd, 0x7d, I2C_ACK_CHECK_EN);
	//i2c_master_write_byte(cmd, 0x12, I2C_ACK_CHECK_EN);
	//i2c_master_write_byte(cmd, 0x34, I2C_ACK_CHECK_EN);
	//i2c_master_stop(cmd);
	//printf("Write LED config - regReset\n");
	//i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, (1000 / portTICK_RATE_MS));	//"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
  //i2c_cmd_link_delete(cmd);

  //if (i2c_ret == ESP_OK) {
  //  printf("Write succeeded.\n");
  //} else {
  //  printf("Write failed.\n");
  //}

  // regInputDisableA
  i2c_write(0x01, 0xff, "regInputDisableA");
	//cmd = i2c_cmd_link_create();
	//i2c_master_start(cmd);
	//i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	//i2c_master_write_byte(cmd, 0x01, I2C_ACK_CHECK_EN);
	//i2c_master_write_byte(cmd, 0xff, I2C_ACK_CHECK_EN);
	//i2c_master_stop(cmd);
	//printf("Write LED config - regInputDisable\n");
	//i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, (1000 / portTICK_RATE_MS));	//"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
  //i2c_cmd_link_delete(cmd);

  //if (i2c_ret == ESP_OK) {
  //  printf("Write succeeded.\n");
  //} else {
  //  printf("Write failed.\n");
  //}

  uint8_t write_addr = (I2C_ADDR<<1) | I2C_WRITE_BIT;
	cmd = i2c_cmd_link_create();
  // regPullDownA
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x09, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0xff, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regOpenDrainA (disabled - push pull operation)
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x0b, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x00, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regDirA
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x0f, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x00, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regClock - set internal oscillator, OSCIO pin is an input
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x1e, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x40, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regMisc - set LED clock to 2Mhz/(2^^3-1)
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x1f, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x30, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regLedDriverEnableA
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x21, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0xff, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regTOn4
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x35, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x0f, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regTIOn4
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x36, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0xff, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regOff4
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x37, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x0f, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regTRise4
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x38, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x0f, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
  // regTFall4
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x39, I2C_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x0f, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	//Send queued commands
	printf("Write LED config\n");
	i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, (1000 / portTICK_RATE_MS));	//"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
  i2c_cmd_link_delete(cmd);

  if (i2c_ret == ESP_OK) {
    printf("Write succeeded.\n");
  } else {
    printf("Write failed.\n");
  }

  while (1) {

    // regDataA
	  cmd = i2c_cmd_link_create();
	  i2c_master_start(cmd);
    i2c_master_write_byte(cmd, write_addr, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x11, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xff, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    //Send queued commands
		printf("Activate LEDs\n");
    i2c_master_cmd_begin(I2C_NUM_1, cmd, (1000 / portTICK_RATE_MS));	//"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);

	  vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void app_main()
{
    //xTaskCreate(&blinky, "blinky", 512,NULL,5,NULL );
    xTaskCreate(&blinken_i2c_test, "blinken_i2c_test", 2048,NULL,5,NULL );
}

