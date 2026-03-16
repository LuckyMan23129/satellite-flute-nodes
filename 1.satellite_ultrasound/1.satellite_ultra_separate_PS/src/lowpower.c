#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>

#include "lowpower.h"
#include "config.h"

LOG_MODULE_REGISTER(lowpower);


static const struct device *const console_dev0 = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct device *const console_dev2 = DEVICE_DT_GET(DT_NODELABEL(uart2));

static const struct device *const uart3_dev = DEVICE_DT_GET(DT_NODELABEL(uart3));

static uint8_t err;

// Disable accel - Reduce 12uA
void lowpower_setup_accel(void)
{
	const struct device *sensor = DEVICE_DT_GET(DT_ALIAS(accel0));

	if (!device_is_ready(sensor))
	{
		LOG_ERR("Could not get accel0 device");
		return;
	}

	// Disable the device
	struct sensor_value odr = {
		.val1 = 0,
	};

	int rc = sensor_attr_set(sensor, SENSOR_CHAN_ACCEL_XYZ,
							 SENSOR_ATTR_SAMPLING_FREQUENCY,
							 &odr);
	if (rc != 0)
	{
		LOG_ERR("Failed to set odr: %d", rc);
		return;
	}
}


// Setup GPIO => help reduce 141uA
int8_t lowpower_setup_gpio(void)
{
	gpio_pin_configure_dt(&sw0, GPIO_DISCONNECTED);
	gpio_pin_configure_dt(&led0, GPIO_DISCONNECTED);
	gpio_pin_configure_dt(&latch_en, GPIO_DISCONNECTED);

	gpio_pin_configure_dt(&wp, GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_configure_dt(&hold, GPIO_INPUT | GPIO_PULL_UP);

	return 0;
}

//----------------------------------------------------------------------------------------------------#
// Disable UART2 and UART console																	  #
//----------------------------------------------------------------------------------------------------#
 	/**
	 * @note: We should NOT turn off UART console and UART2 (sensor interface)  somewhere 
	 * 		  while uart2 (satellite communication) is active to prevent high_speed_clock is turn off
	 */	

// Disable UART console
int8_t lowpower_setup_uart0_DIS(void)
{
	//static const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	/* Disable console UART */
	err = pm_device_action_run(console_dev0, PM_DEVICE_ACTION_SUSPEND);
	if (err < 0)
	{
		LOG_ERR("Unable to suspend console UART. (err: %d)", err);
		return err;
	}

	/* Turn off to save power (High Speed clock) */
	// NRF_CLOCK->TASKS_HFCLKSTOP = 1;

	return 0;
}

// Disable UART2
int8_t lowpower_setup_uart2_DIS(void)
{
	//static const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	/* Disable console UART */
	err = pm_device_action_run(console_dev2, PM_DEVICE_ACTION_SUSPEND);
	if (err < 0)
	{
		LOG_ERR("Unable to suspend UART 2 (err: %d)", err);
		return err;
	}

	/* Turn off to save power (High Speed clock) */
	// NRF_CLOCK->TASKS_HFCLKSTOP = 1;

	return 0;
}


//----------------------------------------------------------------------------------------------------#
// Enable UART2 and UART console																	  #
//----------------------------------------------------------------------------------------------------#
// Enable UART0
int8_t lowpower_setup_uart0_ENA(void)
{
    err = pm_device_action_run(console_dev0, PM_DEVICE_ACTION_RESUME);
    if (err < 0 && err != -EALREADY)
    {
      LOG_ERR("Unable to RESUME console UART (err: %d)", err);
	  return err;
    }
	return 0;
}

// Enable UART2
int8_t lowpower_setup_uart2_ENA(void)
{
    err = pm_device_action_run(console_dev2, PM_DEVICE_ACTION_RESUME);
    if (err < 0 && err != -EALREADY)
    {
      LOG_ERR("Unable to RESUME UART2 (err: %d)", err);
	  return err;
    }
	return 0;
}


//----------------------------------------------------------------------------------------------------#
//  Enable and disable UART3																			  #
//----------------------------------------------------------------------------------------------------#
// Disable UART3
int8_t lowpower_setup_uart3_DIS(void)
{
	/* Disable console UART */
	err = pm_device_action_run(uart3_dev, PM_DEVICE_ACTION_SUSPEND);
	if (err < 0)
	{
		LOG_ERR("Unable to suspend UART3. (err: %d)", err);
		return err;
	}

	/* Turn off to save power (High Speed clock) */
	NRF_CLOCK->TASKS_HFCLKSTOP = 1;

	return 0;
}

// Enable UART3
int8_t lowpower_setup_uart3_ENA(void)
{
    err = pm_device_action_run(uart3_dev, PM_DEVICE_ACTION_RESUME);
    if (err < 0 && err != -EALREADY)
    {
      LOG_ERR("Unable to RESUME UART3 (err: %d)", err);
	  return err;
    }
	return 0;
}