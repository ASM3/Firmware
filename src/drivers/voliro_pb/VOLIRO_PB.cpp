/****************************************************************************
 *
 *   Copyright (c) 2020 Wyss Zurich - ETH Zurich Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file voliro_pb.cpp
 * Driver for the VOLIRO power board connected via I2C.
 *
 * Author: Amir Melzer
 *
 */

#include "VOLIRO_PB.hpp"

using namespace time_literals;

VOLIRO_PB::VOLIRO_PB(I2CSPIBusOption bus_option, int bus, device::Device *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus,
		     interface->get_device_address()),
	_px4_voliro_pb(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors"))
{
}

VOLIRO_PB::~VOLIRO_PB()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
VOLIRO_PB::init()
{

	reset_sensor(); /* dummy I2C command */

	if (reset_sensor() != OK) {
		PX4_DEBUG("reset failed");
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

int
VOLIRO_PB::reset()
{

	return reset_sensor();
}

int
VOLIRO_PB::configure_sensor()
{
	_measurement_res = SHT2x_RES_12_14BIT;				// ToDo: proper allocation

	/* send change resolution command */
	int ret = res_change(_measurement_res);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		_state = State::configure;
		init();
		return ret;
	}

	_state = State::temperature_measurement;

	return ret;
}

int VOLIRO_PB::reset_sensor()
{
	/* send a reset command */
	int ret = cmd_reset();

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	_state = State::configure;

	return ret;
}

void

VOLIRO_PB::start()
{
	/* run at twice the sample rate to capture all new data */
	ScheduleOnInterval(1000000 / VOLIRO_PB_MEAS_RATE / 2);
}

void
VOLIRO_PB::RunImpl()
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


	case State::v_collection:

		/* temperature collection phase */
		if (temperature_collection() == PX4_OK) {

			/* next phase is humidity measurement */
			_state = State::humidity_measurement;
			ScheduleDelayed(10_ms);

		} else {
			/* try to reconfigure */
			_state = State::configure;
		}

		break;


	case State::i_collection:

		/* humidity collection phase */
		if (humidity_colection() == PX4_OK) {

			/* next phase is temperature measurement */
			_state = State::temperature_measurement;

			if (VOLIRO_PB_CONVERSION_INTERVAL - _temp_conversion_time - _hum_conversion_time < 0) {
				ScheduleDelayed(VOLIRO_PB_CONVERSION_INTERVAL);

			} else {
				ScheduleDelayed(VOLIRO_PB_CONVERSION_INTERVAL - _temp_conversion_time - _hum_conversion_time);
			}

		} else {
			/* try to reconfigure */
			_state = State::configure;
		}

		break;
	}
}

int
VOLIRO_PB::v_colection()
{

	sensor_voliro_pb_s report{};

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	/* fetch the raw value */

	uint8_t pwr_brd_status = 0;
	uint8_t pwr_brd_led_status = 0;
	uint8_t	pwr_brd_blink_reg = 0;
	uint8_t	pwr_brd_led_1_pwr = 0;
	uint8_t	pwr_brd_led_2_pwr = 0;
	uint8_t	pwr_brd_led_3_pwr = 0;
	uint8_t	pwr_brd_led_4_pwr = 0;

	float pwr_brd_system_v = 0.0f;
	float pwr_brd_servo_v  = 0.0f;
	float pwr_brd_digital_v  = 0.0f;

	float pwr_brd_mot_l = 0.0f;
	float pwr_brd_mot_r = 0.0f;

	float pwr_brd_analog   = 0.0f;
	float pwr_brd_ext      = 0.0f;
	float pwr_brd_digital = 0.0f;
	float pwr_brd_aux      = 0.0f;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	if (OK != get_regs(LED_STATS_REG, &pwr_brd_led_status)) {
		return -EIO;
	}

	if (OK != get_regs(LED_BLINK_REG, &pwr_brd_blink_reg)) {
		return -EIO;
	}

	if (OK != get_regs(LED_POW_1_REG, &pwr_brd_led_1_pwr)) {
		return -EIO;
	}

	if (OK != get_regs(LED_POW_2_REG, &pwr_brd_led_2_pwr)) {
		return -EIO;
	}

	if (OK != get_regs(LED_POW_3_REG, &pwr_brd_led_3_pwr)) {
		return -EIO;
	}

	if (OK != get_regs(LED_POW_4_REG, &pwr_brd_led_4_pwr)) {
		return -EIO;
	}

	if (OK != get_regs(BRD_STATS_REG, &pwr_brd_status)) {
		return -EIO;
	}

	if (OK != get_voltage(SYSTEM_VOLT_REG, &pwr_brd_system_v)) {
		return -EIO;
	}

	if (OK != get_voltage(SERVO_VOLT_REG, &pwr_brd_servo_v)) {
		return -EIO;
	}

	if (OK != get_voltage(DIGITAL_VOLTAGE_REG, &pwr_brd_digital_v)) {
		return -EIO;
	}

	if (OK != get_channel_current(DIGITAL_CURRENT_REG, &pwr_brd_digital)) {
		return -EIO;
	}

	if (OK != get_channel_current(ANALOG_CURRENT_REG, &pwr_brd_analog)) {
		return -EIO;
	}

	if (OK != get_channel_current(AUX_CURRENT_REG, &pwr_brd_aux)) {
		return -EIO;
	}

	if (OK != get_motor_current(MOT_L_CURRENT_REG, &pwr_brd_mot_l)) {
		return -EIO;
	}

	if (ENA_MOT_R) {
		if (OK != get_motor_current(MOT_R_CURRENT_REG, &pwr_brd_mot_r)) {
			return -EIO;
		}

	} else {
		if (OK != get_motor_current(EXT_CURRENT_REG, &pwr_brd_ext)) {
			return -EIO;
		}
	}

	/* apply calibration values */
	pwr_brd_system_v = (MV_TO_V(pwr_brd_system_v) * SYSTEM_VOLTAGE_SCALE - _scale._bias_cal_term_system_volt) *
			   _scale._SF_cal_term_system_volt;
	pwr_brd_servo_v  = (MV_TO_V(pwr_brd_servo_v) * SERVO_VOLTAGE_SCALE - _scale._bias_cal_term_servo_volt) *
			   _scale._SF_cal_term_servo_volt;
	pwr_brd_digital_v = (MV_TO_V(pwr_brd_digital_v) * DIGITAL_VOLTAGE_SCALE - _scale._bias_cal_term_digital_volt) *
			    _scale._SF_cal_term_digital_volt;

	pwr_brd_mot_l = (pwr_brd_mot_l - _scale._bias_cal_term_mot_l_amp) * _scale._SF_cal_term_mot_l_amp;
	pwr_brd_mot_r = (pwr_brd_mot_r - _scale._bias_cal_term_mot_r_amp) * _scale._SF_cal_term_mot_r_amp;

	pwr_brd_digital = (pwr_brd_digital - _scale._bias_cal_term_digital_amp) * _scale._SF_cal_term_digital_amp;
	pwr_brd_analog = (pwr_brd_analog - _scale._bias_cal_term_analog_amp) * _scale._SF_cal_term_analog_amp;
	pwr_brd_ext = (pwr_brd_ext - _scale._bias_cal_term_ext_amp) * _scale._SF_cal_term_ext_amp;
	pwr_brd_aux	= (pwr_brd_aux - _scale._bias_cal_term_aux_amp) * _scale._SF_cal_term_aux_amp;

	/* generate a new report */
	prb.timestamp = hrt_absolute_time();

	prb.pwr_brd_status  	= pwr_brd_status;
	prb.pwr_brd_led_status  = pwr_brd_led_status;

	prb.pwr_brd_blink_reg  = pwr_brd_blink_reg;
	prb.pwr_brd_led_1_pwr  = pwr_brd_led_1_pwr;
	prb.pwr_brd_led_2_pwr  = pwr_brd_led_2_pwr;
	prb.pwr_brd_led_3_pwr  = pwr_brd_led_3_pwr;
	prb.pwr_brd_led_4_pwr  = pwr_brd_led_4_pwr;

	prb.pwr_brd_system_volt = pwr_brd_system_v;
	prb.pwr_brd_servo_volt  = pwr_brd_servo_v;
	prb.pwr_brd_digital_volt = pwr_brd_digital_v;

	prb.pwr_brd_mot_l_amp 	= pwr_brd_mot_l;
	prb.pwr_brd_mot_r_amp 	= pwr_brd_mot_r;

	prb.pwr_brd_digital_amp  = pwr_brd_digital;
	prb.pwr_brd_analog_amp   = pwr_brd_analog;
	prb.pwr_brd_ext_amp      = pwr_brd_ext;
	prb.pwr_brd_aux_amp  	 = pwr_brd_aux;


	/* generate a new report */
	if (!FILTERVALUES) {
		report.relative_humidity = _relative_humidity;					    /* report in percent */
		report.ambient_temperature = _temperature;						    /* report in cel */

	} else {
		report.relative_humidity = _filter_v.apply(_relative_humidity);	/* filtered report in percent */
		report.ambient_temperature = _filter_i.apply(_temperature);		/* filtered report in degc    */
	}

	_px4_voliro_pb.set_error_count(perf_event_count(_comms_errors));
	_px4_voliro_pb.update(timestamp_sample, (float) report.relative_humidity, (float) report.ambient_temperature);

	//PX4_INFO("VOLIRO_PB: Temperature is: %3.2f C, humidity value is: %3.2f", (double) _temperature, (double) _relative_humidity);

	perf_end(_sample_perf);

	return OK;
}

int
PWR_BRD::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		collect();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

/* Get registers value */
int
PWR_BRD::get_regs(uint8_t ptr, uint8_t *regs)
{
	uint8_t data[2];

	if (OK != transfer(&ptr, 1, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*regs  = data[1];

	return OK;
}

/* Set registers value */
int
PWR_BRD::set_regs(uint8_t ptr, uint8_t value)
{
	uint8_t data[2];

	data[0] = ptr;
	data[1] = value;

	if (OK != transfer(&data[0], 2, nullptr, 0)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* wait between transfers*/
	usleep(10);

	/* read back the reg and verify */

	if (OK != transfer(&ptr, 1, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (data[1] != value) {
		return -EIO;
	}

	return OK;
}

/* Set LED power */
int
PWR_BRD::set_LED_power(uint8_t ptr, uint8_t pwr_brd_led_pwr)
{
	/* input power -> scale to percentage 0 -100% */

	pwr_brd_led_pwr = (uint8_t)((float)pwr_brd_led_pwr * 2.55f);

	if (OK != set_regs(ptr, pwr_brd_led_pwr)) {
		return -EIO;
	}

	return OK;
}

/* Set LED blink interval in sec*/
int
PWR_BRD::set_LED_blink_interval(uint8_t blink_interval_sec)
{
	uint8_t ptr = LED_BLINK_REG;
	uint8_t led_blink_Reg;

	if (OK != get_regs(ptr, &led_blink_Reg)) {
		return -EIO;
	}

	led_blink_Reg = ((led_blink_Reg << 4) & 0xf0) | (blink_interval_sec & 0x0f);

	if (OK != set_regs(LED_BLINK_REG, led_blink_Reg)) {
		return -EIO;
	}

	return OK;
}

/* Set LED number of blinks */
int
PWR_BRD::set_LED_number_of_blinks(uint8_t blink_number)
{
	uint8_t ptr = LED_BLINK_REG;
	uint8_t led_blink_Reg;

	if (OK != get_regs(ptr, &led_blink_Reg)) {
		return -EIO;
	}

	led_blink_Reg = ((blink_number << 4) & 0xf0) | (led_blink_Reg & 0x0f);

	if (OK != set_regs(LED_BLINK_REG, led_blink_Reg)) {
		return -EIO;
	}

	return OK;
}

/* Set/Reset LED control remote mode*/
int
PWR_BRD::set_remote_mode_LED_control(bool remote_led_enable)
{
	uint8_t ptr = BRD_STATS_REG;
	uint8_t status_reg;

	if (OK != get_regs(ptr, &status_reg)) {
		return -EIO;
	}

	if (remote_led_enable) {
		status_reg |= (1 << LED_REMOTE_MODE_BIT);

	} else {
		status_reg &= ~(1 << LED_REMOTE_MODE_BIT);
	}

	if (OK != set_regs(BRD_STATS_REG, status_reg)) {
		return -EIO;
	}

	return OK;
}

/* Get system/servo voltage */
int
PWR_BRD::get_voltage(uint8_t ptr, float *voltage)
{
	uint8_t data[3];

	if (OK != transfer(&ptr, 1, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*voltage = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1])) * VOLTAGE_FULLSCALE / ADC_FULLSCALE;

	return OK;
}

/* Get channel current */
int
PWR_BRD::get_channel_current(uint8_t ptr, float *channel_current)
{
	uint8_t data[3];

	if (OK != transfer(&ptr, 1, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*channel_current = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1]) - ADC_FULLSCALE / 10.0f) * VOLTAGE_FULLSCALE /
			   ADC_FULLSCALE / CHANNEL_CURRENT_CONV_FARTOR;

	return OK;
}

/* Get motor current */
int
PWR_BRD::get_motor_current(uint8_t ptr, float *motor_current)
{
	uint8_t data[3];

	if (OK != transfer(&ptr, 1, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (ptr == MOT_L_CURRENT_REG) {
		*motor_current = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1]) - ADC_FULLSCALE / 10.0f) * VOLTAGE_FULLSCALE /
				 ADC_FULLSCALE / MOTOR_L_CURRENT_CONV_FARTOR;

	} else if (ptr == MOT_R_CURRENT_REG) {
		*motor_current = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1]) - ADC_FULLSCALE / 2.0f) * VOLTAGE_FULLSCALE /
				 ADC_FULLSCALE / MOTOR_R_CURRENT_CONV_FARTOR;

	} else if (ptr == EXT_CURRENT_REG) {
		*motor_current = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1]) - ADC_FULLSCALE / 2.0f) * VOLTAGE_FULLSCALE /
				 ADC_FULLSCALE / EXT_CURRENT_CONV_FARTOR;

	} else {
		return -EIO;
	}

	return OK;
}

void
PWR_BRD::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_good_transfers);
	_reports->print_info("pwr_brd queue");

	printf("offsets system voltage (%.2f)\n", (double)_scale._bias_cal_term_system_volt);
	printf("scaling system voltage (%.2f)\n", (double)_scale._SF_cal_term_system_volt);

	printf("offsets mot L current (%.2f)\n", (double)_scale._bias_cal_term_mot_l_amp);
	printf("scaling mot L current(%.2f)\n", (double)_scale._SF_cal_term_mot_l_amp);

	printf("offsets mot R current(%.2f)\n", (double)_scale._bias_cal_term_mot_r_amp);
	printf("scaling mot R current(%.2f)\n", (double)_scale._SF_cal_term_mot_r_amp);

	printf("offsets servo voltage (%.2f)\n", (double)_scale._bias_cal_term_servo_volt);
	printf("scaling servo voltage (%.2f)\n", (double)_scale._SF_cal_term_servo_volt);

	printf("offsets analog current (%.2f)\n", (double)_scale._bias_cal_term_analog_amp);
	printf("scaling analog current (%.2f)\n", (double)_scale._SF_cal_term_analog_amp);

	printf("offsets digital voltage (%.2f)\n", (double)_scale._bias_cal_term_digital_volt);
	printf("scaling digital voltage (%.2f)\n", (double)_scale._SF_cal_term_digital_volt);

	printf("offsets digital current (%.2f)\n", (double)_scale._bias_cal_term_digital_amp);
	printf("scaling system current (%.2f)\n", (double)_scale._SF_cal_term_digital_amp);

	printf("offsets extension current (%.2f)\n", (double)_scale._bias_cal_term_ext_amp);
	printf("scaling extension current (%.2f)\n", (double)_scale._SF_cal_term_ext_amp);

	printf("offsets aux current (%.2f)\n", (double)_scale._bias_cal_term_aux_amp);
	printf("scaling aux current (%.2f)\n", (double)_scale._SF_cal_term_aux_amp);

	printf("\n");
}

int
VOLIRO_PB::cmd_reset()
{
	uint8_t		cmd = RESET_CMD;									/* trigger sensor reset */
	int		result;

	//result = _interface->write((uint8_t)cmd, nullptr, 0);

	// make sure to wait 15ms after configuring the measurement mode
	//ScheduleDelayed(15_ms);

	return result;
}

void
VOLIRO_PB::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
