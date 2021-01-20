/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file sht2x.cpp
 * Driver for the SHT2X humidity and temperature sensor connected via I2C.
 *
 * Author: Amir Melzer
 *
 */

#include "SHT2X.hpp"

using namespace time_literals;

SHT2X::SHT2X(I2CSPIBusOption bus_option, int bus, device::Device *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus,
		     interface->get_device_address()),
	 _px4_hum_temp(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors"))
{
}

SHT2X::~SHT2X()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
SHT2X::init(){

	return configure_sensor() == 0;

}

int
SHT2X::reset(){

	return reset_sensor() == 0;

}

int
SHT2X::configure_sensor()
{
	_measurement_res = SHT2x_RES_12_14BIT;				/// temp find other place for the init

	/* send change resolution command */
	int ret = res_change(_measurement_res);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		//DEVICE_DEBUG("config failed");
		_state = State::configure;
		return ret;
	}

	_state = State::configure;

	//_device_id.devid_s.devtype = DRV_HUM_TEMP_DEVTYPE_SHT2X;

	return ret;
}

int SHT2X::reset_sensor()
{
	/* send a reset command */
	int ret = cmd_reset();

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		//DEVICE_DEBUG("reset failed");
		return ret;
	}

	_state = State::configure;

	return ret;
}

void
SHT2X::start()
{
	// make sure to wait 10ms after configuring the measurement mode
	ScheduleDelayed(10_ms);
}

void
SHT2X::RunImpl()
{

	switch (_state) {

	case State::configure:
		/* sensor configure phase */
		if (configure_sensor() == PX4_OK) {
			ScheduleDelayed(10_ms);
		} else {
			/* periodically retry to configure */
			ScheduleDelayed(300_ms);
		}

		break;

	case State::temperature_measurement:
		/* temperature measurement phase */
		if (temperature_measurement() == PX4_OK) {

			/* next phase is temperate collection */
			_state = State::temperature_collection;
			ScheduleDelayed(_temp_conversion_time);
		} else {
			/* try to reconfigure */
			_state = State::configure;
		}

		break;

	case State::temperature_collection:

		/* temperature collection phase */
		if (temperature_collection() == PX4_OK) {

			/* next phase is humidity measurement */
			_state = State::humidity_measurement;
		} else {
			/* try to reconfigure */
			_state = State::configure;
		}

		break;

	case State::humidity_measurement:
		/* humidity measurement phase */
		if (humidity_measurement() == PX4_OK) {

			/* next phase is humidity collection */
			_state = State::humidity_collection;
			ScheduleDelayed(_hum_conversion_time);
		} else {
			/* try to reconfigure */
			_state = State::configure;
		}

		break;

	case State::humidity_collection:
		/* humidity collection phase */
		if (humidity_colection() == PX4_OK) {

			/* next phase is temperature measurement */
			_state = State::temperature_measurement;
			if (SHT2X_CONVERSION_INTERVAL - _temp_conversion_time - _hum_conversion_time < 0) {
				ScheduleDelayed(SHT2X_CONVERSION_INTERVAL);
			}
			else{
				ScheduleDelayed(SHT2X_CONVERSION_INTERVAL - _temp_conversion_time - _hum_conversion_time);
			}
		} else {
			/* try to reconfigure */
			_state = State::configure;
		}

		break;
	}
}

int
SHT2X::humidity_measurement()
{
	uint8_t cmd = TRIG_RH_MEASUREMENT_POLL;							/* Trigger humidity measurement  */

	//if (OK != transfer(&cmd, 1, nullptr, 0)) {
	//	return -EIO;
	//}

	if (_interface->write(cmd, nullptr, 1) != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -EIO;
	}

	return OK;
}

int
SHT2X::humidity_colection()
{
	uint8_t data[3];
	union {
		uint8_t	b[3];
		uint32_t w;
	} cvt;

	sensor_hum_temp_s report{};

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();

	/* fetch the raw value */

	//if (OK != transfer(nullptr, 0, &data[0], 3)) {
	if (OK != 	_interface->read(0, &data, 3))
	{
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[1];
	cvt.b[1] = data[0];
	cvt.b[2] = data[2];

	/* calculate CRC and return success/failure accordingly */
	if (!crc8(&cvt.b[0])) {
		return -EIO;
	}

	/* Relative humidity calculation, result in percent (corresponds to the relative humidity above liquid water)  */
	_relative_humidity = -6.0f + 125.0f / 65536.0f * ((float)((uint16_t)(cvt.w & 0xffff)));

	/* Range check failure accordingly */
	if ((_relative_humidity <= 0) | (_relative_humidity >= 100)) {
		PX4_INFO("SHT2X: humidity value is out of range: %3.6f", (double) _relative_humidity);
		_relative_humidity = -1000.0f;
		return -EIO;
	}
	/* generate a new report */
	if (!FILTERVALUES) {
		report.relative_humidity = _relative_humidity;					    /* report in percent */
		report.ambient_temperature = _temperature;						    /* report in cel */

	} else {
		report.relative_humidity = _filter_hum.apply(_relative_humidity);	/* filtered report in percent */
		report.ambient_temperature = _filter_temp.apply(_temperature);		/* filtered report in degc    */
	}

	report.error_count = perf_event_count(_comms_errors);

	_px4_hum_temp.update(report.timestamp, report.relative_humidity,report.ambient_temperature);


	perf_end(_sample_perf);

	return OK;
}

int
SHT2X::temperature_measurement()
{
	uint8_t cmd = TRIG_T_MEASUREMENT_POLL;							  /* trigger temperature measurement */

	//if (OK != transfer(&cmd, 1, nullptr, 0)) {
	//	return -EIO;
	//}
	if (_interface->write(cmd, nullptr, 1) != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -EIO;
	}

	return OK;
}

int
SHT2X::temperature_collection()
{
	uint8_t data[3];
	union {
		uint8_t	b[3];
		uint32_t w;
	} cvt;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* fetch the raw value */
	//if (OK != transfer(nullptr, 0, &data[0], 3)) {
	//	perf_count(_comms_errors);
	//	return -EIO;
	//}

	if (OK != _interface->read(0, &data, 3))
	{
		perf_count(_comms_errors);
		return -EIO;
	}

	/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
	cvt.b[0] = data[1];
	cvt.b[1] = data[0];
	cvt.b[2] = data[2];

	/* calculate CRC and return success/failure accordingly */
	if (!crc8(&cvt.b[0])) {
		return -EIO;
	}

	/* temperature calculation, result in C */
	_temperature = -46.85f + 175.72f / 65536.0f * ((float)((int16_t)(cvt.w & 0xffff)));

	if ((_temperature > 100) | (_temperature < -40)) {
		PX4_INFO("SHT2X: Temperature is out of range: %3.2f C", (double) _temperature);
		_temperature = -1000.0f;
		return -EIO;
	}

	perf_end(_sample_perf);

	return OK;
}

int
SHT2X::cmd_reset()
{
	uint8_t		cmd = RESET_CMD;									/* trigger sensor reset */
	int		result;

	//result = transfer(&cmd, 1, nullptr, 0);
	result = _interface->write(cmd, nullptr, 1);
	return result;
}

int
SHT2X::res_change(uint8_t res)
{
	uint8_t		cmd = USER_REG_R;
	uint8_t 	data[2];


	//if (OK != transfer(&cmd, 1, &data[0], 1)) {
	if (OK != _interface->read(cmd, &data, 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	cmd 	= USER_REG_W;

	data[1] = (data[0] | res);
	data[0] = cmd;

	//if (OK != transfer(&data[0], 2, nullptr, 0)) {
	if (OK != _interface->write(cmd, data, 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	switch (res) {
	case SHT2x_RES_12_14BIT: {
			_temp_conversion_time = SHT2x_RES_14BIT_TEMP_CONVERSION;
			_hum_conversion_time = SHT2x_RES_12BIT_HUM_CONVERSION;
			break;
		}

	case SHT2x_RES_8_12BIT: {
			_temp_conversion_time = SHT2x_RES_12BIT_TEMP_CONVERSION;
			_hum_conversion_time = SHT2x_RES_8BIT_HUM_CONVERSION;
			break;
		}

	case SHT2x_RES_10_13BIT: {
			_temp_conversion_time = SHT2x_RES_13BIT_TEMP_CONVERSION;
			_hum_conversion_time = SHT2x_RES_10BIT_HUM_CONVERSION;
			break;
		}

	case SHT2x_RES_11_11BIT: {
			_temp_conversion_time = SHT2x_RES_11BIT_TEMP_CONVERSION;
			_hum_conversion_time = SHT2x_RES_11BIT_HUM_CONVERSION;
			break;
		}

	default: {
			_temp_conversion_time = SHT2X_MAX_TEMP_CONVERSION;
			_hum_conversion_time = SHT2X_MAX_HUM_CONVERSION;
			break;
		}
	}

	return OK;
}

bool
SHT2X::crc8(uint8_t *crc_data)
{
	uint8_t crc_read;
	uint8_t crc = 0;
	uint8_t byteCtr;
	uint8_t numberOfDataBytes = 2;

	/* save the read crc */
	crc_read = crc_data[numberOfDataBytes];

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = numberOfDataBytes; byteCtr > 0; --byteCtr) {
		crc ^= (crc_data[byteCtr - 1]);

		for (uint8_t bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ CRC_POLYNOM;

			} else {
				crc = (crc << 1);
			}
		}
	}

	return (crc_read == crc);
}

void
SHT2X::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
