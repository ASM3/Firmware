/****************************************************************************
 *
 *   Copyright (c) 2021 Wyss Zurich - ETH Zurich Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file VOLIRO_PB.h
 *
 * @author Amir Melzer
 *
 */

#pragma once

#include <math.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <drivers/device/Device.hpp>
#include <lib/drivers/voliro_pb/PX4Voliro_pb.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/i2c_spi_buses.h>


/* VOLIRO_PB internal constants and data structures.																*/
#define VOLIRO_PB_SLAVE_ADDRESS	0x49      /* Power board I2C address												*/
#define VOLIRO_PB_BUS_SPEED     100*1000  /* Power board I2C bus speed 												*/

#define LED_STATS_REG			0x00	  /* Pointer to the address of the led board status register 				*/
#define LED_BLINK_REG			0x01	  /* Pointer to the address of the led blink register  						*/
#define LED_POW_1_REG			0x02	  /* Pointer to the address of the led power register of channel 1 			*/
#define LED_POW_2_REG			0x03	  /* Pointer to the address of the led power register of channel 2  		*/
#define LED_POW_3_REG			0x04	  /* Pointer to the address of the led power register of channel 3  		*/
#define LED_POW_4_REG			0x05	  /* Pointer to the address of the led power register of channel 4  		*/
#define BRD_STATS_REG			0x06	  /* Pointer to the address of the board status register 					*/
#define SYSTEM_VOLT_REG			0x07	  /* Pointer to the address of board system voltage register			  	*/
#define SYSTEM_CURRENT_REG		0x09	  /* Pointer to the address of board system current register				*/
#define BAT_VOLT_REG			0x0b	  /* Pointer to the address of board battery voltage register			  	*/
#define BAT_CURRENT_REG			0x0d	  /* Pointer to the address of board battery current register			  	*/
#define DIGITAL_5V_CURRENT_REG	0x0f	  /* Pointer to the address of board 5v digital current register			*/
#define ANALOG_5V_CURRENT_REG	0x11	  /* Pointer to the address of board 5v analog current register				*/
#define DIGITAL_12V_CURRENT_REG	0x13	  /* Pointer to the address of board 12v digital current register			*/
#define ANALOG_12V_CURRENT_REG	0x15	  /* Pointer to the address of board 12v analog current register			*/

#define LED_REMOTE_MODE_BIT     0x4		  /* LED remote mode bit on the status register		   						*/

#define ADC_FULLSCALE				1024.0f
#define VOLTAGE_FULLSCALE			3300.0f
#define CHANNEL_CURRENT_CONV_FARTOR	132.0f
#define MOTOR_L_CURRENT_CONV_FARTOR	66.0f
#define MOTOR_R_CURRENT_CONV_FARTOR	33.0f
#define EXT_CURRENT_CONV_FARTOR	    33.0f
#define SYSTEM_VOLTAGE_SCALE		9.333f
#define SERVO_VOLTAGE_SCALE			2.0f
#define DIGITAL_VOLTAGE_SCALE		2.0f

#define FILTERVALUES			  false /* filtering measured values 				*/

/* Measurement rate is 5Hz (verify maximum measurement rate according to the mode)  */
#define VOLIRO_PB_MEAS_RATE 5
#define MEAS_DRIVER_FILTER_FREQ 2.0f
#define VOLIRO_PB_CONVERSION_INTERVAL	(1000000 / VOLIRO_PB_MEAS_RATE)		/* microseconds */

/* arithmetic helper macro: convert millivolt to volt */
#define MV_TO_V(_x)		( (_x) * 0.001f )

#define SETLEDPOWER 1

class VOLIRO_PB : public I2CSPIDriver<VOLIRO_PB>
{
public:
	VOLIRO_PB(I2CSPIBusOption bus_option, const int bus, device::Device *interface);
	virtual ~VOLIRO_PB();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int			init();

	void		print_status();

	void		RunImpl();

private:
	enum class State {
		configure,
		burst_collection
	};

	void start();
	int			reset();

	PX4Voliro_pb		_px4_voliro_pb;

	device::Device		*_interface;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	State 				_state;

	//struct voliro_pb_calibration_s    _scale;

	float _bias_cal_term_system_volt;
	float _SF_cal_term_system_volt;

	float _bias_cal_term_servo_volt;
	float _SF_cal_term_servo_volt;

	float _bias_cal_term_mot_l_amp;
	float _SF_cal_term_mot_l_amp;

	float _bias_cal_term_mot_r_amp;
	float _SF_cal_term_mot_r_amp;

	float _bias_cal_term_digital_amp;
	float _SF_cal_term_digital_amp;
	float _bias_cal_term_analog_amp;
	float _SF_cal_term_analog_amp;
	float _bias_cal_term_ext_amp;
	float _SF_cal_term_ext_amp;
	float _bias_cal_term_digital_volt;
	float _SF_cal_term_digital_volt;

	float _bias_cal_term_aux_amp;
	float _SF_cal_term_aux_amp;

	unsigned _pwr_brd_led_blink_int;
	unsigned _pwr_brd_led_blink_number;
	unsigned _pwr_brd_led_remote_mode;
	unsigned _pwr_brd_led_power_1;
	unsigned _pwr_brd_led_power_2;
	unsigned _pwr_brd_led_power_3;
	unsigned _pwr_brd_led_power_4;

	math::LowPassFilter2p _filter_v{VOLIRO_PB_MEAS_RATE, MEAS_DRIVER_FILTER_FREQ};
	//math::LowPassFilter2p _filter_i{VOLIRO_PB_MEAS_RATE, MEAS_DRIVER_FILTER_FREQ};

	/**
	 * Perform power board configuration.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			configure_sensor();

	/**
	 * Perform power board reset.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			reset_sensor();

	/**
	 * Print power board calibration parameters.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			print_info();

	/**
	 * Issue a sensor data burst collection command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			burst_collection();

	/**
	 * Issue a sensor self test command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			self_test();

	/**
	 * Get power board registers values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_regs(uint8_t ptr, uint8_t *regs);

	/**
	 * Set power board registers values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			set_regs(uint8_t ptr, uint8_t value);

	/**
	 * Set LEDs power values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			set_LED_power(uint8_t ptr, uint8_t pwr_brd_led_pwr);

	/**
	 * Set LEDs blink interval in sec
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			set_LED_blink_interval(uint8_t blink_interval_sec);

	/**
	 * Set LEDs number of blinks
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			set_LED_number_of_blinks(uint8_t blink_number);

	/**
	 * Set/Reset LED remote mode
	 *
	 * @return		OK if the command setting was successful.
	 */
	int			set_remote_mode_LED_control(bool remote_led_enable);

	/**
	 * Get system/servo voltage values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_voltage(uint8_t ptr, float *voltage);

	/**
	 * Get power board channel current values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_channel_current(uint8_t ptr, float *channel_current);

	/**
	 * Get power board motor current values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_motor_current(uint8_t ptr, float *motor_current);

	/**
	 * Get power board data
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			get_pwr_brd_data();

};
