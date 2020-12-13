/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/pcnt.h"

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

#define GPIO_SW_BUTTON 39
#define GPIO_SW_ROT_B 36
#define GPIO_SW_ROT_A 35

#define GPIO_SW_PIN_SEL  ((1ULL<<GPIO_SW_BUTTON) | (1ULL<<GPIO_SW_ROT_A) | (1ULL<<GPIO_SW_ROT_B))
#define ESP_INTR_FLAG_DEFAULT 0

static void IRAM_ATTR pcnt_example_intr_handler(void *arg) {
	uint32_t intr_status = PCNT.int_st.val;

	for (int i = 0; i < PCNT_UNIT_MAX; i++) {
		if (intr_status & (BIT(i))) {

			if(PCNT.status_unit[i].h_lim_lat){
				printf("Hit high limit\n");
			}
			if(PCNT.status_unit[i].l_lim_lat){
				printf("Hit low limit\n");
			}
			//pcnt_counter_clear(ptr->unit);
			PCNT.int_clr.val = BIT(i); // clear the interrupt
		}
	}
}


void rotary_input() {
	gpio_pad_select_gpio(GPIO_SW_ROT_A);
	gpio_pad_select_gpio(GPIO_SW_ROT_B);
	gpio_set_direction(GPIO_SW_ROT_A, GPIO_MODE_INPUT);
	gpio_set_direction(GPIO_SW_ROT_B, GPIO_MODE_INPUT);
  gpio_pullup_en(GPIO_SW_ROT_A);
	gpio_pullup_en(GPIO_SW_ROT_B);

  pcnt_config_t r_enc_config;
  // channel 0
  r_enc_config.pulse_gpio_num = GPIO_SW_ROT_A; //Rotary Encoder Chan A
	r_enc_config.ctrl_gpio_num = GPIO_SW_ROT_B;    //Rotary Encoder Chan B

	r_enc_config.unit = PCNT_UNIT_0;
	r_enc_config.channel = PCNT_CHANNEL_0;

	r_enc_config.pos_mode = PCNT_COUNT_DIS; // PCNT_COUNT_DEC;
	r_enc_config.neg_mode = PCNT_COUNT_INC;

	r_enc_config.lctrl_mode = PCNT_MODE_KEEP;    // Rising A on HIGH B = CW Step
	r_enc_config.hctrl_mode = PCNT_MODE_REVERSE; // Rising A on LOW B = CCW Step

	r_enc_config		.counter_h_lim = 255;
	r_enc_config		.counter_l_lim = 0;

	pcnt_unit_config(&r_enc_config);

  // channel 1 - disabled for half-quad encoder
  r_enc_config.pulse_gpio_num = GPIO_SW_ROT_A; //Rotary Encoder Chan A
	r_enc_config.ctrl_gpio_num = GPIO_SW_ROT_B;    //Rotary Encoder Chan B
	r_enc_config.channel = PCNT_CHANNEL_1;
	r_enc_config.pos_mode = PCNT_COUNT_DIS; //PCNT_COUNT_INC;
	r_enc_config.neg_mode = PCNT_COUNT_DIS; //PCNT_COUNT_DEC;
	r_enc_config.lctrl_mode = PCNT_MODE_DISABLE;
	r_enc_config.hctrl_mode = PCNT_MODE_DISABLE;
	pcnt_unit_config(&r_enc_config);

  pcnt_set_filter_value(PCNT_UNIT_0, 1023);  // Filter Runt Pulses
	pcnt_filter_enable(PCNT_UNIT_0);

  /* Enable events on maximum and minimum limit values */
	//pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
	//pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

	pcnt_counter_pause(PCNT_UNIT_0); // Initial PCNT init
	pcnt_counter_clear(PCNT_UNIT_0);

	/* Register ISR handler and enable interrupts for PCNT unit */
  //esp_err_t er = pcnt_isr_register(pcnt_example_intr_handler,(void *) NULL, (int)0, NULL);
  //if (er != ESP_OK){
  //  printf("Encoder wrap interrupt failed\n");
  //}
  //
  //pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_example_intr_handler, (void *)NULL);
	pcnt_intr_enable(PCNT_UNIT_0);
	pcnt_counter_resume(PCNT_UNIT_0);

  int16_t val=0, oldval=0;
  while (1) {
    pcnt_get_counter_value(PCNT_UNIT_0, &val);
    if (val != oldval) {
      printf("%20s %d\n", "Encoder value", val);
    }

    oldval = val;

	  vTaskDelay(100 / portTICK_RATE_MS);
  }

}

void app_main()
{
    xTaskCreate(&blinken_i2c_test, "blinken_i2c_test", 2048,NULL,5,NULL );
    xTaskCreate(&rotary_input, "rotary_input", 2048,NULL,5,NULL );
}

