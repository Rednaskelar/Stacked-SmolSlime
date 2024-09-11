#include "globals.h"
#include "sys.h"
//#include "timer.h"
//#include "util.h"
#include "esb.h"
#include "sensor.h"
#include "battery.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
#if IGNORE_RESET && DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios)
	bool reset_pin_reset = false;
#else
	bool reset_pin_reset = NRF_POWER->RESETREAS & 0x01;
#endif
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
	gpio_pin_configure_dt(&dock, GPIO_INPUT);
#endif

	power_check(); // check the battery and dock first before continuing (4ms to read from ADC)

//	start_time = k_uptime_get(); // Need to get start time for imu startup delay
	set_led(SYS_LED_PATTERN_ON); // Boot LED

#if CONFIG_BUILD_OUTPUT_UF2 && !(IGNORE_RESET && DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios)) // Using Adafruit bootloader
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
	ram_range_retain(dbl_reset_mem, sizeof(dbl_reset_mem), true);
#endif

	uint8_t reboot_counter = reboot_counter_read();
	bool booting_from_shutdown = reboot_counter == 0; // 0 means from user shutdown or failed ram validation;

	if (booting_from_shutdown)
		set_led(SYS_LED_PATTERN_ONESHOT_POWERON);

	if (reset_pin_reset || button_read()) // Count pin resets
	{
		if (reboot_counter == 0)
			reboot_counter = 100;
		else if (reboot_counter > 200)
			reboot_counter = 200; // How did you get here
		reset_mode = reboot_counter - 100;
		reboot_counter++;
		reboot_counter_write(reboot_counter);
		LOG_INF("Reset count: %u", reboot_counter);
		k_msleep(1000); // Wait before clearing counter and continuing
		reboot_counter_write(100);
		if (!reset_pin_reset && !button_read() && reset_mode == 0) // Only need to check once, if the button is pressed again an interrupt is triggered from before
			reset_mode = -1; // Cancel reset_mode (shutdown)
	}
// 0ms or 1000ms for reboot counter

#if USER_SHUTDOWN_ENABLED
	if (reset_mode == 0 && !booting_from_shutdown) // Reset mode user shutdown
	{
		LOG_INF("User shutdown requested");
		reboot_counter_write(0);
		set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF);
		// TODO: scheduled power off
		k_msleep(1500);
		if (button_read()) // If alternate button is available and still pressed, wait for the user to stop pressing the button
		{
			set_led(SYS_LED_PATTERN_LONG);
			while (button_read())
				k_msleep(1);
			set_led(SYS_LED_PATTERN_OFF);
		}
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
		bool docked = gpio_pin_get_dt(&dock);
#else
		bool docked = false;
#endif
		if (!docked) // TODO: should the tracker start again if docking state changes?
			configure_system_off_chgstat();
		else
			configure_system_off_dock(); // usually charging, i would flash LED but that will drain the battery while it is charging..
	}
// How long user shutdown take does not matter really ("0ms")
#endif

	sys_read();
// 5-6ms to initialize NVS (only done when needed)

	set_led(SYS_LED_PATTERN_OFF);

	if (reset_mode == 1)
	{
		LOG_INF("IMU calibration requested");
	}

	if (reset_mode == 2) // Reset mode pairing reset
	{
		LOG_INF("Pairing reset requested");
		esb_reset_pair();
		reset_mode = 0; // Clear reset mode
	}

#if CONFIG_BUILD_OUTPUT_UF2 // Using Adafruit bootloader
	if (reset_mode == 3 || reset_mode == 4) // DFU_MAGIC_UF2_RESET, Reset mode DFU
	{
		LOG_INF("DFU requested");
		NRF_POWER->GPREGRET = 0x57;
		sys_reboot(SYS_REBOOT_COLD);
	}
#endif

	clocks_start();

	esb_pair();

	esb_initialize();
	tx_payload.noack = false;
	//timer_init();
// 1ms to start ESB

	unsigned int last_batt_pptt[16] = {10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001};
	int8_t last_batt_pptt_i = 0;

	while (1)
	{
		// Get start time
		int64_t time_begin = k_uptime_get();

		//charging = gpio_pin_get_dt(&chgstat); // TODO: Charging detect doesn't work (hardware issue)
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
		bool docked = gpio_pin_get_dt(&dock);
#else
		bool docked = false;
#endif

		// TODO: move battery stuff to sys
		int batt_mV;
		batt_pptt = read_batt_mV(&batt_mV);

		bool battery_available = batt_mV > 1500; // Keep working without the battery connected, otherwise it is obviously too dead to boot system

		if (battery_available && batt_pptt == 0 && !docked)
			configure_system_off_chgstat();
		last_batt_pptt[last_batt_pptt_i] = batt_pptt;
		last_batt_pptt_i++;
		last_batt_pptt_i %= 15;
		for (uint8_t i = 0; i < 15; i++)
		{ // Average battery readings across 16 samples
			if (last_batt_pptt[i] == 10001)
				batt_pptt += batt_pptt / (i + 1);
			else
				batt_pptt += last_batt_pptt[i];
		}
		batt_pptt /= 16;
		if (batt_pptt + 100 < last_batt_pptt[15]) // Lower bound -100pptt
			last_batt_pptt[15] = batt_pptt + 100;
		else if (batt_pptt > last_batt_pptt[15]) // Upper bound +0pptt
			last_batt_pptt[15] = batt_pptt;
		else // Effectively 100-10000 -> 1-100%
			batt_pptt = last_batt_pptt[15];

		// format for packet send
		batt = batt_pptt / 100;
		if (battery_available && batt < 1) // Clamp to 1% (because server sees 0% as "no battery")
			batt = 1;
		batt_mV /= 10;
		batt_mV -= 245;
		if (batt_mV < 0) // Very dead but it is what it is
			batt_v = 0;
		else if (batt_mV > 255)
			batt_v = 255;
		else
			batt_v = batt_mV; // 0-255 -> 2.45-5.00V

		if (docked) // TODO: keep sending battery state while plugged and docked?
		// TODO: move to interrupts? (Then you do not need to do the above)
			configure_system_off_dock();

		if (system_off_main) // System off on extended no movement
			configure_system_off_WOM();

		main_data = false;

		wait_for_threads(); // TODO:
		main_imu_wakeup();
		threads_running = true;

		// Get time elapsed and sleep/yield until next tick
		int64_t time_delta = k_uptime_get() - time_begin;
		led_clock_offset += time_delta;
		if (time_delta > tickrate)
			k_yield();
		else
			k_msleep(tickrate - time_delta);
	}
}
