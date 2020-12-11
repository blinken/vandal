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


//void blinky(void *pvParameter)
//{
//
//    gpio_pad_select_gpio(BLINK_GPIO);
//    /* Set the GPIO as a push/pull output */
//    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
//    while(1) {
//        /* Blink off (output low) */
//        gpio_set_level(BLINK_GPIO, 0);
//        vTaskDelay(1000 / portTICK_RATE_MS);
//        /* Blink on (output high) */
//        gpio_set_level(BLINK_GPIO, 1);
//        vTaskDelay(1000 / portTICK_RATE_MS);
//    }
//}

#define I2C_SCL_GPIO				16
#define I2C_SDA_GPIO				17
#define I2C_FREQ_HZ					400000
#define I2C_ADDR            0x3e
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
	printf("%20s (0x%02x) = 0x%02x: ", description, reg, value);
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
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = 17;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = 16;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_FREQ_HZ;
	i2c_param_config(I2C_NUM_1, &conf);
	i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0);

  for (int i=0; i<5; i++) {
    printf("Waiting to start\n");
    vTaskDelay(1000 / portTICK_RATE_MS);
  }

  i2c_write(0x01, 0xff, "regInputDisableA");
  i2c_write(0x09, 0xff, "regPullDownA");
  i2c_write(0x0b, 0x00, "regOpenDrainA"); // disabled - push pull operation
  i2c_write(0x0f, 0x00, "regDirA");
  i2c_write(0x1e, 0x40, "regClock");
  i2c_write(0x1f, 0x30, "regMisc"); // set LED clock to 2Mhz/(2^^3-1)
  i2c_write(0x21, 0xff, "regLedDriverEnableA");
  i2c_write(0x0d, 0x00, "regPolarityA"); // explicitly set polarities - on startup, these seem to be inverted (0xff)
  i2c_write(0x0c, 0x00, "regPolarityB");

  // MOSFET1 - breathe when off
  i2c_write(0x35, 0x0f, "regTOn4");
  i2c_write(0x36, 0xff, "regTIOn4");
  i2c_write(0x37, 0x0f, "regOff4");
  i2c_write(0x38, 0x0f, "regTRise4");
  i2c_write(0x39, 0x0f, "regTFall4");

  // MOSFET2 - breathe when off
  i2c_write(0x3a, 0x0f, "regTOn5");
  i2c_write(0x3b, 0xff, "regTIOn5");
  i2c_write(0x3c, 0x0f, "regOff5");
  i2c_write(0x3d, 0x0f, "regTRise5");
  i2c_write(0x3e, 0x0f, "regTFall5");

  // MOSFET3 - "one-shot mode" - fade off then back on when turned off
  i2c_write(0x3f, 0x0f, "regTOn6");
  i2c_write(0x40, 0xff, "regTIOn6");
  i2c_write(0x41, 0x00, "regOff6");
  i2c_write(0x42, 0x0f, "regTRise6");
  i2c_write(0x43, 0x0f, "regTFall6");

  // MOSFET4 - fade on, fade off
  i2c_write(0x44, 0x00, "regTOn7");
  i2c_write(0x45, 0xff, "regTIOn7");
  i2c_write(0x46, 0x00, "regOff7");
  i2c_write(0x47, 0x0f, "regTRise7");
  i2c_write(0x48, 0x0f, "regTFall7");

  // I/O pins on outputs 0-3 are on-off without fades

  uint8_t state = 0xff;
  i2c_write(0x11, state, "regDataA");
  while (1) {
    i2c_write(0x11, (state=~state), "regDataA");
	  vTaskDelay(10000 / portTICK_RATE_MS);
  }
}

void app_main()
{
    xTaskCreate(&blinken_i2c_test, "blinken_i2c_test", 2048,NULL,5,NULL );
}

