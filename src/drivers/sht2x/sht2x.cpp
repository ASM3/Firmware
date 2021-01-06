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

#include "sht2x.h"

/** driver 'main' command */
extern "C" { __EXPORT int sht2x_main(int argc, char *argv[]); }

namespace sht2x
{

SHT2X *g_dev_ext;
SHT2X *g_dev_int;

void start();
void stop();
void test();
void reset();
void info();
void usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void start()
{
	int fd;
	SHT2X **g_dev_ptr = &g_dev_ext;
	const char *path = HUM_TEMP_DEVICE_PATH;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		PX4_ERR("already started");
		exit(0);
	}

	/* create the driver */
#if defined(PX4_I2C_BUS_EXPANSION)
	*g_dev_ptr = new SHT2X(PX4_I2C_BUS_EXPANSION, path);
#else
	PX4_ERR("External I2C not available");
	exit(0);
#endif

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}


	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path, O_RDONLY);


	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	PX4_ERR("driver start failed");
	exit(1);

}

void stop()
{
	SHT2X **g_dev_ptr = &g_dev_ext;

	if (*g_dev_ptr != nullptr) {
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;

	} else {
		/* warn, but not an error */
		warnx("sht2x sensor already stopped");
	}

	exit(0);
}

void test()
{
	int fd = -1;
	const char *path = HUM_TEMP_DEVICE_PATH;
	struct hum_temp_report p_report;
	ssize_t sz;


	/* get the driver */
	fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'sht2x start' if the driver is not running)",
			path);
		exit(1);
	}

	/* reset to Max polling rate*/
	//if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX) < 0) {
	//	PX4_ERR("reset to Max polling rate");
	//	exit(1);
	//}

	/* do a simple demand read */
	sz = read(fd, &p_report, sizeof(p_report));

	if (sz != sizeof(p_report)) {
		PX4_ERR("immediate sht2x read failed");
		exit(1);
	}

	PX4_INFO("single read");
	PX4_INFO("time:                  %lld", p_report.timestamp);
	PX4_INFO("relative humidity:  %10.4f", (double)p_report.relative_humidity);
	PX4_INFO("temperature:        %10.4f", (double)p_report.ambient_temperature);

	PX4_INFO("PASS");
	exit(0);
}

void
reset()
{
	const char *path = HUM_TEMP_DEVICE_PATH;
	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		exit(1);
	}

	exit(0);

}

void
info()
{
	SHT2X **g_dev_ptr = &g_dev_ext;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);

}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'test', 'stop', 'reset'");
	PX4_INFO("options:");
}

} // namespace sht2x

SHT2X :: SHT2X(int bus, const char *path) :
	I2C("SHT2X", path, bus, SHT2X_SLAVE_ADDRESS, SHT2X_BUS_SPEED),
	_running(false),
	_call_interval(0),
	_reports(nullptr),
	_measurement_res(SHT2x_RES_12_14BIT),
	_acquisition_phase(0),
	_temp_conversion_time(SHT2X_MAX_TEMP_CONVERSION),
	_hum_conversion_time(SHT2X_MAX_HUM_CONVERSION),
	_relative_humidity(0),
	_temperature(0),
	_hum_temp_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "sht2x_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "sht2x_bad_transfers")),
	_good_transfers(perf_alloc(PC_COUNT, "sht2x_good_transfers")),
	_measure_perf(perf_alloc(PC_ELAPSED, "sht2x_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "sht2x_comms_errors")),
	_duplicates(perf_alloc(PC_COUNT, "sht2x_duplicates")),
	_got_duplicate(false),
	_filter_hum(SHT2X_MEAS_RATE, MEAS_DRIVER_FILTER_FREQ),
	_filter_temp(SHT2X_MEAS_RATE, MEAS_DRIVER_FILTER_FREQ)
{
	_device_id.devid_s.devtype = DRV_HUM_TEMP_DEVTYPE_SHT2X;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

}

SHT2X :: ~SHT2X()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_hum_temp_topic != nullptr) {
		orb_unadvertise(_hum_temp_topic);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_good_transfers);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_duplicates);

}

int SHT2X::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("I2C setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(hum_temp_report));

	if (_reports == nullptr) {
		goto out;
	}

	/* do temperature measuremnt */
	if (OK != temperature_measurement()) {
		ret = -EIO;
		return ret;
	}

	usleep(SHT2X_MAX_TEMP_CONVERSION);

	if (OK != temperature_collection()) {
		ret = -EIO;
		return ret;
	}

	/* do humidity measurement */
	if (OK != humidity_measurement()) {
		ret = -EIO;
		return ret;
	}

	usleep(SHT2X_MAX_HUM_CONVERSION);

	if (OK != humidity_colection()) {
		ret = -EIO;
		return ret;
	}

	/* advertise sensor topic, measure manually to initialize valid report */
	struct hum_temp_report prb;
	_reports->get(&prb);

	/* measurement will have generated a report, publish */
	_hum_temp_topic = orb_advertise(ORB_ID(sensor_hum_temp), &prb);

	if (_hum_temp_topic == nullptr) {
		PX4_WARN("ADVERT FAIL");
	}

out:
	return ret;
}

int
SHT2X::probe()
{
	_retries = 10;

	if (OK == conf_sensor()) {
		_retries = 1;
		return OK;
	}

	return -EIO;
}

int
SHT2X::conf_sensor()
{
	/* send change resolution command */
	if (OK != res_change(_measurement_res)) {
		return -EIO;
	}

	return OK;
}

void
SHT2X::start()
{
	/* reset the report ring and state machine */
	_running = false;
	_reports->flush();
	_acquisition_phase = 0;

	if (OK != cmd_reset()) {
		stop();
	}

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&SHT2X::cycle_trampoline, this, USEC2TICK(SHT2X_CONVERSION_INTERVAL));
}

void
SHT2X::stop()
{
	_running = false;
	work_cancel(HPWORK, &_work);
}

ssize_t
SHT2X::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct hum_temp_report);
	struct hum_temp_report *rbuf = reinterpret_cast<struct hum_temp_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* Take a temperature and humidity measurements */

		/* do temperature measurement */
		if (OK != temperature_measurement()) {
			ret = -EIO;
			break;
		}

		usleep(SHT2X_MAX_TEMP_CONVERSION);

		if (OK != temperature_collection()) {
			ret = -EIO;
			break;
		}


		/* do humidity measurement */
		if (OK != humidity_measurement()) {
			ret = -EIO;
			break;
		}

		usleep(SHT2X_MAX_HUM_CONVERSION);

		if (OK != humidity_colection()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
SHT2X::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, SHT2X_MAX_DATA_RATE);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, SHT2X_MEAS_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(SHT2X_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCRESET:
		reset();
		return OK;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

int SHT2X::reset()
{
	/* send a reset command */
	if (OK != cmd_reset()) {
		return -EIO;
	}

	return OK;
}

void
SHT2X::cycle_trampoline(void *arg)
{
	SHT2X *dev = reinterpret_cast<SHT2X *>(arg);

	/* make measurement */
	dev->cycle();
}

void
SHT2X::cycle()
{
	int32_t conersion_interval = 0;

	if (!_running) {
		if (OK != conf_sensor()) {
			start();
		}

		_running = true;
	}

	switch (_acquisition_phase) {

	case 0: {	/* temperate measurement phase */
			if (OK != temperature_measurement()) {
				start();
				return;
			}

			/* next phase is temperate collection */
			_acquisition_phase = 1;
			conersion_interval = _temp_conversion_time;
			break;
		}

	case 1: {	/* temperature collection phase */
			if (OK != temperature_collection()) {
				start();
				return;
			}

			/* next phase is humidity measurement */
			_acquisition_phase = 2;
			break;
		}

	case 2: {	/* humidity measurement phase */
			if (OK != humidity_measurement()) {
				start();
				return;
			}

			/* next phase is humidity collection */
			_acquisition_phase = 3;
			conersion_interval = _hum_conversion_time;
			break;
		}

	case 3: {	/* humidity collection phase */
			if (OK != humidity_colection()) {
				start();
				return;
			}

			/* next phase is temperature measurement */
			_acquisition_phase = 0;
			conersion_interval  = SHT2X_CONVERSION_INTERVAL - _temp_conversion_time - _hum_conversion_time;

			if (conersion_interval < 0) {
				conersion_interval = SHT2X_CONVERSION_INTERVAL;
			}

			break;
		}

	default: {
			_acquisition_phase = 0;
			conersion_interval  = SHT2X_CONVERSION_INTERVAL;
			break;
		}
	}

	/* schedule a fresh cycle call when the measurement is done */
	unsigned wait_gap = USEC2TICK(conersion_interval);
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&SHT2X::cycle_trampoline,
		   this,
		   wait_gap);
}

int
SHT2X::humidity_measurement()
{
	uint8_t cmd = TRIG_RH_MEASUREMENT_POLL;							/* Trigger humidity measurement  */

	if (OK != transfer(&cmd, 1, nullptr, 0)) {
		return -EIO;
	}

	return OK;
}

int
SHT2X::humidity_colection()
{
	bool sht2x_notify = true;

	uint8_t data[3];
	union {
		uint8_t	b[3];
		uint32_t w;
	} cvt;

	hum_temp_report trp;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	trp.timestamp = hrt_absolute_time();

	/* fetch the raw value */

	if (OK != transfer(nullptr, 0, &data[0], 3)) {
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
		trp.relative_humidity = _relative_humidity;					    /* report in percent */
		trp.ambient_temperature = _temperature;						    /* report in cel */

	} else {
		trp.relative_humidity = _filter_hum.apply(_relative_humidity);	/* filtered report in percent */
		trp.ambient_temperature = _filter_temp.apply(_temperature);		/* filtered report in degc    */
	}

	_reports->force(&trp);

	/* notify anyone waiting for data */
	if (sht2x_notify) {
		poll_notify(POLLIN);
	}

	if (sht2x_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_hum_temp), _hum_temp_topic, &trp);
	}

	perf_end(_sample_perf);
	return OK;
}

int
SHT2X::temperature_measurement()
{
	uint8_t cmd = TRIG_T_MEASUREMENT_POLL;							  /* trigger temperature measurement */

	if (OK != transfer(&cmd, 1, nullptr, 0)) {
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
	if (OK != transfer(nullptr, 0, &data[0], 3)) {
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

	result = transfer(&cmd, 1, nullptr, 0);

	return result;
}

int
SHT2X::res_change(uint8_t res)
{
	uint8_t		cmd = USER_REG_R;
	uint8_t 	data[2];


	if (OK != transfer(&cmd, 1, &data[0], 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	cmd 	= USER_REG_W;

	data[1] = (data[0] | res);
	data[0] = cmd;

	if (OK != transfer(&data[0], 2, nullptr, 0)) {
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
SHT2X::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_good_transfers);
	_reports->print_info("sht2x queue");

	printf("Relative humidity:     %10.4f\n", (double)(_relative_humidity));
	printf("Ambient temperature:   %10.4f\n", (double)(_temperature));
	printf("\n");
}

int
sht2x_main(int argc, char *argv[])
{

	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "")) != EOF) {
		switch (ch) {

		default:
			sht2x::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		sht2x::start();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "stop")) {
		sht2x::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		sht2x::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		sht2x::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		sht2x::info();
	}

	sht2x::usage();
	exit(1);
}
