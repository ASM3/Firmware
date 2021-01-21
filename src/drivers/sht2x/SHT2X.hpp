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
 * @file SHT2X.h
 *
 * @author Amir Melzer
 *
 */

#pragma once

#include <math.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <drivers/device/Device.hpp>
#include <lib/drivers/hum_temp/PX4Hum_Temp.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/i2c_spi_buses.h>


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

class SHT2X : public I2CSPIDriver<SHT2X>
{
public:
	SHT2X(I2CSPIBusOption bus_option, const int bus, device::Device *interface);
	virtual ~SHT2X();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int			init();

	void		print_status();

	void		RunImpl();

private:
	enum class State {
		configure,
		temperature_measurement,
		temperature_collection,
		humidity_measurement,
		humidity_collection
	};

	void start();
	int			reset();

	PX4Hum_Temp			_px4_hum_temp;

	device::Device		*_interface;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	int 				_measurement_res;
	State 				_state;

	int 				_temp_conversion_time;
	int 				_hum_conversion_time;

	float 				_relative_humidity;
	float 				_temperature;

	math::LowPassFilter2p _filter_hum{SHT2X_MEAS_RATE, MEAS_DRIVER_FILTER_FREQ};
	math::LowPassFilter2p _filter_temp{SHT2X_MEAS_RATE, MEAS_DRIVER_FILTER_FREQ};

	/**
	 * Perform SHT2X sensor configuration.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				configure_sensor();

	/**
	 * Perform SHT2X sensor reset.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				reset_sensor();

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
};
