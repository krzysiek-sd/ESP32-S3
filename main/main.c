/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <esp_system.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "hal/mcpwm_ll.h"
#include "driver/mcpwm_cap.h"

#define TIMER_FREQ    1000000	//1MHz so resolution 1us
#define SYS_TIMER_PERIOD TIMER_FREQ/1000		//1ms
#define Z_MEAS_2P_TIMER_PERIOD TIMER_FREQ/100	//100us

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)


#define BUF_SIZE (1024)

#define SPI_ADS131A02_CLK_FREQ_HZ			(1*20000*1000)   //1MHz - min stable SCLK for ADS131A02 is 25MHz

#define I2C_LCD_EXPANDER_MASTER_NUM 		I2C_NUM_0          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_LCD_EXPANDER_MASTER_FREQ_HZ		800000             /*!< I2C master clock frequency */
#define I2C_LCD_EXPANDER_MASTER_SDA_IO		35
#define I2C_LCD_EXPANDER_MASTER_SCL_IO		36

#define I2C_OTHER_DEVICES_MASTER_NUM 		I2C_NUM_1          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_OTHER_DEVICES_MASTER_FREQ_HZ	10000              /*!< I2C master clock frequency */
#define I2C_OTHER_DEVICES_MASTER_SDA_IO		47
#define I2C_OTHER_DEVICES_MASTER_SCL_IO		48
#define I2C_MASTER_TIMEOUT_MS				1000

#define I2C_MASTER_RX_BUF_DISABLE			0				/*!< I2C master doesn't need buffer */
#define I2C_MASTER_TX_BUF_DISABLE			0				/*!< I2C master doesn't need buffer */

#define ESP_INTR_FLAG_DEFAULT 0

#define GPIO_ADC_SPI_CS   			11
#define GPIO_ADC_DRDY		   		13
#define GPIO_SYNCH_H				4
#define GPIO_SYNCH_L				5
#define GPIO_SYNCH_V0_A				6

#define GPIO_POWER_LOCK				2
#define GPIO_POWER_BUTTON			1

static const char *TAG = "TEST";


static bool IRAM_ATTR SysTimerISRcallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
	BaseType_t high_task_awoken = pdFALSE;

	return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

static bool IRAM_ATTR Synch_L_isr_handler(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
	bool ret = 0;

    return ret;
}

static bool IRAM_ATTR Synch_H_isr_handler(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
	bool ret = 0;

    return ret;
}


static bool IRAM_ATTR Synch_V0_isr_handler(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
	bool ret = 0;

    return ret;
}

static void ADC_SetCS(void)
{
	gpio_set_level(14, 1);
}

static void ADC_ClrCS(void)
{
	gpio_set_level(14, 1);
}

void app_main(void)
{

	//=================================== I2C init =========================================================
	i2c_config_t conf;

	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_LCD_EXPANDER_MASTER_SDA_IO;
	conf.scl_io_num = I2C_LCD_EXPANDER_MASTER_SCL_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_LCD_EXPANDER_MASTER_FREQ_HZ;
	conf.clk_flags = 0;
	i2c_param_config(I2C_LCD_EXPANDER_MASTER_NUM, &conf);
	i2c_driver_install (I2C_LCD_EXPANDER_MASTER_NUM, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);


	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_OTHER_DEVICES_MASTER_SDA_IO;
	conf.scl_io_num = I2C_OTHER_DEVICES_MASTER_SCL_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_OTHER_DEVICES_MASTER_FREQ_HZ;
	conf.clk_flags = 0;
	i2c_param_config(I2C_OTHER_DEVICES_MASTER_NUM, &conf);
	i2c_driver_install (I2C_OTHER_DEVICES_MASTER_NUM, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

	//========================= GPIO init ================================

	gpio_set_direction(GPIO_ADC_DRDY, GPIO_MODE_INPUT);

	gpio_set_direction(GPIO_ADC_SPI_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_ADC_SPI_CS, 1);

	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

	//================================ Timer1 init ===========================

	gptimer_handle_t gptimerSysTimer1 = NULL;

	gptimer_config_t timer_config1 =
	{
	   .clk_src = GPTIMER_CLK_SRC_DEFAULT,
	   .direction = GPTIMER_COUNT_UP,
	   .resolution_hz = TIMER_FREQ, 			// 1MHz, 1 tick=1us
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config1, &gptimerSysTimer1));

	gptimer_event_callbacks_t cbs1 =
	{
		.on_alarm = SysTimerISRcallback,
	};

	ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimerSysTimer1, &cbs1, NULL));

	ESP_ERROR_CHECK(gptimer_enable(gptimerSysTimer1));

	gptimer_alarm_config_t alarm_config1 =
	{
		.alarm_count = SYS_TIMER_PERIOD, //period = 1ms
		.flags.auto_reload_on_alarm = true,
	};


	ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimerSysTimer1, &alarm_config1));
	ESP_ERROR_CHECK(gptimer_start(gptimerSysTimer1));

	//================================ Timer2 init ===========================

	gptimer_handle_t gptimerSysTimer2 = NULL;

	gptimer_config_t timer_config2 =
	{
	   .clk_src = GPTIMER_CLK_SRC_DEFAULT,
	   .direction = GPTIMER_COUNT_UP,
	   .resolution_hz = TIMER_FREQ, 			// 1MHz, 1 tick=1us
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config2, &gptimerSysTimer2));

	gptimer_event_callbacks_t cbs2 =
	{
		.on_alarm = SysTimerISRcallback,
	};

	ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimerSysTimer2, &cbs2, NULL));

	ESP_ERROR_CHECK(gptimer_enable(gptimerSysTimer2));

	gptimer_alarm_config_t alarm_config2 =
	{
		.alarm_count = SYS_TIMER_PERIOD, //period = 1ms
		.flags.auto_reload_on_alarm = true,
	};

	ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimerSysTimer2, &alarm_config2));

	//============================== SPI init ========================================

	spi_device_handle_t ADS131A02Spi2DeviceHandle;    ///< SPI2 device handle

	spi_bus_config_t spiBusConf =			//this kind of initialization clear other values in structure
	{
		.miso_io_num = 10,
		.mosi_io_num = 11,
		.sclk_io_num = 12,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 32,
	};

	spi_bus_initialize(SPI2_HOST, &spiBusConf, SPI_DMA_CH_AUTO);

	spi_device_interface_config_t spiDevConf =
	{
		.command_bits = 0,
		.clock_speed_hz = SPI_ADS131A02_CLK_FREQ_HZ,
		.mode = 1,	//SPI mode 1 => CPOL = 0, CPHA = 1
		.spics_io_num = -1,
		.queue_size = 1,
		.flags = SPI_DEVICE_NO_DUMMY,
		.pre_cb =  (void*)ADC_ClrCS,			//cs_high,
		.post_cb = (void*)ADC_SetCS,			//cs_low,
		.input_delay_ns = 0,//SPI_ADS131A02_INPUT_DELAY_NS,  //the EEPROM output the data half a SPI clock behind - nie wiem czy to potrzebne
	};

	spi_bus_add_device(SPI2_HOST, &spiDevConf, &ADS131A02Spi2DeviceHandle);

	//=================================== Capture init ========================================

	mcpwm_capture_channel_config_t conf_SYNCH_L = {0};
	mcpwm_capture_channel_config_t conf_SYNCH_H = {0};
	mcpwm_capture_channel_config_t conf_SYNCH_V0 = {0};

	mcpwm_cap_timer_handle_t cap_timer = NULL;
	mcpwm_capture_timer_config_t cap_conf = {
	    .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
	    .group_id = 0,
	};

	mcpwm_cap_channel_handle_t cap_SYNCH_L_chan = NULL;
	mcpwm_cap_channel_handle_t cap_SYNCH_H_chan = NULL;
	mcpwm_cap_channel_handle_t cap_SYNCH_V0_chan = NULL;

	ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

	//---------------------------- set CAP_0 on GPIO_SYNCH_L ---------------------------------
	conf_SYNCH_L.gpio_num = GPIO_SYNCH_L;
	conf_SYNCH_L.prescale = 1;
	conf_SYNCH_L.flags.neg_edge = false;
	conf_SYNCH_L.flags.pos_edge = true;
	conf_SYNCH_L.flags.pull_up = true;
	conf_SYNCH_L.flags.pull_down = false;

	ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &conf_SYNCH_L, &cap_SYNCH_L_chan));

	TaskHandle_t task_synch_L = xTaskGetCurrentTaskHandle();

	mcpwm_capture_event_callbacks_t cbs_synch_L =
	{
		.on_cap = Synch_L_isr_handler,
	};

	ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_SYNCH_L_chan, &cbs_synch_L, task_synch_L));
	//-------------------------------------------------------------------------------------------------

	//---------------------------- set CAP_1 on GPIO_SYNCH_H -------------------------------------
	conf_SYNCH_H.gpio_num = GPIO_SYNCH_H;
	conf_SYNCH_H.prescale = 1;
	conf_SYNCH_H.flags.neg_edge = false;
	conf_SYNCH_H.flags.pos_edge = true;
	conf_SYNCH_H.flags.pull_up = true;
	conf_SYNCH_H.flags.pull_down = false;

	ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &conf_SYNCH_H, &cap_SYNCH_H_chan));

	mcpwm_capture_event_callbacks_t cbs_synch_H =
	{
		.on_cap = Synch_H_isr_handler,
	};

	ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_SYNCH_H_chan, &cbs_synch_H, NULL));
	//-------------------------------------------------------------------------------------------------

	//----------------------------- set CAP_2 on GPIO_SYNCH_V0 --------------------------------------
	conf_SYNCH_V0.gpio_num = GPIO_SYNCH_V0_A;
	conf_SYNCH_V0.prescale = 1;
	conf_SYNCH_V0.flags.neg_edge = false;
	conf_SYNCH_V0.flags.pos_edge = true;
	conf_SYNCH_V0.flags.pull_up = true;
	conf_SYNCH_V0.flags.pull_down = false;

	ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &conf_SYNCH_V0, &cap_SYNCH_V0_chan));

	mcpwm_capture_event_callbacks_t cbs_synch_V0 =
	{
		.on_cap = Synch_V0_isr_handler,
	};

	ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_SYNCH_V0_chan, &cbs_synch_V0, NULL));
	//-------------------------------------------------------------------------------------------------

	ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
	ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

	//=======================================================================================================

	while(1)
	{
		ESP_LOGI(TAG, "Tick 500");
		vTaskDelay(pdMS_TO_TICKS(500));
		taskYIELD();
	}

	//===================================== UART init ==========================================================

	uart_config_t uart_config =
	{
		.baud_rate = ECHO_UART_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};
	int intr_alloc_flags = 0;

	#if CONFIG_UART_ISR_IN_IRAM
	    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
	#endif

	ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

	while(1)
	{
		ESP_LOGI(TAG, "Tick 500");
		vTaskDelay(pdMS_TO_TICKS(500));
		taskYIELD();
	}

	//=======================================================================================================
}
