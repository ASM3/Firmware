/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sht2x.h
 *
 * @author Amir Melzer
 *
 */

#pragma once

#include <math.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <drivers/drv_hum_temp.h>


/* SHT2X internal constants and data structures.									*/

#define SHT2X_BUS				  PX4_I2C_BUS_EXPANSION
#define SHT2X_SLAVE_ADDRESS		  0x40
#define SHT2X_MAX_DATA_RATE       10
#define SHT2X_BUS_SPEED           100*1000

#define TRIG_T_MEASUREMENT_HM     0xE3 	/* trigger a temp meas. hold master			*/
#define TRIG_RH_MEASUREMENT_HM    0xE5 	/* trigger a humidity meas. hold master		*/
#define TRIG_T_MEASUREMENT_POLL   0xF3	/* trigger a temp meas. no hold master		*/
#define TRIG_RH_MEASUREMENT_POLL  0xF5 	/* trigger a humidity meas. no hold master	*/
#define USER_REG_W                0xE6 	/* write to user register					*/
#define USER_REG_R                0xE7 	/* read from user register					*/
#define RESET_CMD	              0xFE  /* command to reset chip 					*/

#define SHT2x_RES_12_14BIT        0x00 	/* RH=12bit, T=14bit						*/
#define SHT2x_RES_8_12BIT         0x01 	/* RH= 8bit, T=12bit						*/
#define SHT2x_RES_10_13BIT        0x80	/* RH=10bit, T=13bit						*/
#define SHT2x_RES_11_11BIT        0x81  /* RH=11bit, T=11bit						*/

#define SHT2x_EOB_ON              0x40	/* end of battery							*/

#define SHT2x_HEATER_ON           0x04 	/* heater on								*/
#define SHT2x_HEATER_OFF          0x00	/* heater off								*/

#define CRC_POLYNOM 			  0x131 /* define the SHT2X CRC8 Polynomial: P(x)=x^8+x^5+x^4+1 = 100110001 */

#define FILTERVALUES			  false /* filtering measured values 				*/

/* Measurement rate is 5Hz (verify maximum measurement rate according to the mode)  */
#define SHT2X_MEAS_RATE 5
#define MEAS_DRIVER_FILTER_FREQ 2.0f
#define SHT2X_CONVERSION_INTERVAL	(1000000 / SHT2X_MEAS_RATE)		/* microseconds */

#define SHT2X_MAX_TEMP_CONVERSION			85000					/* microseconds */
#define SHT2X_MAX_HUM_CONVERSION			29000					/* microseconds */
#define SHT2x_RES_11BIT_TEMP_CONVERSION		11000					/* microseconds */
#define SHT2x_RES_12BIT_TEMP_CONVERSION		22000					/* microseconds */
#define SHT2x_RES_13BIT_TEMP_CONVERSION		43000					/* microseconds */
#define SHT2x_RES_14BIT_TEMP_CONVERSION		85000					/* microseconds */
#define SHT2x_RES_8BIT_HUM_CONVERSION		4000					/* microseconds */
#define SHT2x_RES_10BIT_HUM_CONVERSION		9000					/* microseconds */
#define SHT2x_RES_11BIT_HUM_CONVERSION		15000					/* microseconds */
#define SHT2x_RES_12BIT_HUM_CONVERSION		29000					/* microseconds */

class SHT2X : public device::I2C
{
public:
	SHT2X(int bus, const char *path);
	virtual ~SHT2X();

	virtual int         init();
	virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
	virtual int       	ioctl(struct file *filp, int cmd, unsigned long arg);


	/* Diagnostics - print some basic information about the driver. */
	void                print_info();

protected:
	virtual int         probe();

private:
	work_s            _work{};

	bool _running;

	/* altitude conversion calibration */
	unsigned        _call_interval;

	hum_temp_report _report {};
	ringbuffer::RingBuffer  *_reports;

	unsigned			_measurement_res;
	int					_acquisition_phase;

	int					_temp_conversion_time;
	int					_hum_conversion_time;

	float 				_relative_humidity;
	float 				_temperature;

	orb_advert_t        _hum_temp_topic;

	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _good_transfers;
	perf_counter_t      _measure_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _duplicates;

	bool            	_got_duplicate;

	math::LowPassFilter2p	_filter_hum;
	math::LowPassFilter2p	_filter_temp;

	/**
	 * Command set sensor initialization.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				conf_sensor();

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Reset the automatic measurement state machine.
	 */
	int 			reset();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a humidity measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				humidity_measurement();

	/**
	 * Issue a temperature measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				temperature_measurement();

	/**
	 * Issue a humidity collection command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				humidity_colection();

	/**
	 * Issue a temperature collection command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				temperature_collection();

	/**
	 * Send a reset command to the SHT2X.
	 *
	 * This is required after any bus reset.
	 */
	int				cmd_reset();

	/**
	 * Set measurement resolution for the SHT2X.
	 *
	 */
	int 			res_change(uint8_t res);

	/**
	 * CRC routine ported from SHT2X application note
	 *
	 * @param n_prom	Pointer to words read from PROM.
	 * @return		True if the CRC matches.
	 */
	bool			crc8(uint8_t *n_prom);

	/* do not allow to copy this class due to pointer data members */
	SHT2X(const SHT2X &);
	SHT2X operator=(const SHT2X &);

};
