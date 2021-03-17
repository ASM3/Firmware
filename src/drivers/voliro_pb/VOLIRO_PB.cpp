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
	// default scaling
	_scale._bias_cal_term_system_volt = 0.0;
	_scale._SF_cal_term_system_volt = 1.0;
	_scale._bias_cal_term_system_amp = 0.0;
	_scale._SF_cal_term_system_amp = 1.0;
	_scale._bias_cal_term_battery_volt = 0.0;
	_scale._SF_cal_term_battery_volt = 1.0;
	_scale._bias_cal_term_battery_amp = 0.0;
	_scale._SF_cal_term_battery_amp = 1.0;
	_scale._bias_cal_term_5v_digital_amp = 0.0;
	_scale._SF_cal_term_5v_digital_amp = 1.0;
	_scale._bias_cal_term_5v_analog_amp = 0.0;
	_scale._SF_cal_term_5v_analog_amp = 1.0;
	_scale._bias_cal_term_12v_digital_amp = 0.0;
	_scale._SF_cal_term_12v_digital_amp = 1.0;
	_scale._bias_cal_term_12v_analog_amp = 0.0;
	_scale._SF_cal_term_12v_analog_amp = 1.0;
	// default settings
	_pwr_brd_led_status = 0;
	_pwr_brd_led_blink_int = 0x1;
	_pwr_brd_led_mask = 0xf;
	_pwr_brd_led_power_1 = 100;
	_pwr_brd_led_power_2 = 80;
	_pwr_brd_led_power_3 = 60;
	_pwr_brd_led_power_4 = 40;
}

VOLIRO_PB::~VOLIRO_PB()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

/* Sensor initialization */
int
VOLIRO_PB::init()
{

	reset_sensor(); /* dummy I2C command */

	if (reset_sensor() != OK) {
		PX4_DEBUG("reset failed");
		return PX4_ERROR;
	}

	// Getting initial parameter values
	update_params();

	start();

	return PX4_OK;
}

int
VOLIRO_PB::reset()
{

	return reset_sensor();
}

/* Sensor configuration */
int
VOLIRO_PB::configure_sensor()
{
	uint8_t ret = PX4_OK;

	if (OK != set_LED_blink_interval(_pwr_brd_led_blink_int)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != set_LED_mask(_pwr_brd_led_mask)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != set_LED_power(LED_POW_1_REG, _pwr_brd_led_power_1)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != set_LED_power(LED_POW_2_REG, _pwr_brd_led_power_2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != set_LED_power(LED_POW_3_REG, _pwr_brd_led_power_3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (OK != set_LED_power(LED_POW_4_REG, _pwr_brd_led_power_4)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	return ret;
}
/* Sensor reset */
int VOLIRO_PB::reset_sensor()
{

	uint8_t ret = PX4_OK;

	/* send a reset command */
	// ToDo: implement a reset function on the PB side

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

			/* next phase is burst collection measurement */
			_state = State::burst_collection;

		} else {
			/* periodically retry to configure */
			ScheduleDelayed(300_ms);
		}

		break;

	case State::burst_collection:

		/* temperature collection phase */
		if (burst_collection() == PX4_OK) {

			/* next phase is humidity measurement */
			_state = State::burst_collection;

			ScheduleDelayed(VOLIRO_PB_CONVERSION_INTERVAL);

		} else {
			/* try to reconfigure */
			_state = State::configure;
			/* periodically retry to configure */
			ScheduleDelayed(300_ms);
		}

		break;
	}
}
/* Burst data collection */
int
VOLIRO_PB::burst_collection()
{

	sensor_voliro_pb_s report{};

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp_sample = hrt_absolute_time();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();
	}


	/* fetch the raw value */

	uint8_t pwr_brd_status = 0;
	uint8_t pwr_brd_led_status = 0;
	uint8_t	pwr_brd_blink_reg = 0;
	uint8_t	pwr_brd_led_1_pwr = 0;
	uint8_t	pwr_brd_led_2_pwr = 0;
	uint8_t	pwr_brd_led_3_pwr = 0;
	uint8_t	pwr_brd_led_4_pwr = 0;

	float pwr_brd_system_v = 0.0f;
	float pwr_brd_system_i = 0.0f;

	float pwr_brd_bat_v = 0.0f;
	float pwr_brd_bat_i = 0.0f;

	float pwr_5v_digital_i  = 0.0f;
	float pwr_5v_analog_i  = 0.0f;

	float pwr_12v_digital_i  = 0.0f;
	float pwr_12v_analog_i  = 0.0f;

	/* read the most recent measurement */
	perf_begin(_sample_perf);

	if (OK != get_regs(LED_STATS_REG, &pwr_brd_led_status)) {
		return -EIO;
	}

	if (OK != get_regs(LED_REG, &pwr_brd_blink_reg)) {
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

	if (OK != get_voltage(SYSTEM_CURRENT_REG, &pwr_brd_system_i)) {
		return -EIO;
	}

	if (OK != get_voltage(BAT_VOLT_REG, &pwr_brd_bat_v)) {
		return -EIO;
	}

	if (OK != get_voltage(BAT_CURRENT_REG, &pwr_brd_bat_i)) {
		return -EIO;
	}

	if (OK != get_channel_current(DIGITAL_5V_CURRENT_REG, &pwr_5v_digital_i)) {
		return -EIO;
	}

	if (OK != get_channel_current(ANALOG_5V_CURRENT_REG, &pwr_5v_analog_i)) {
		return -EIO;
	}

	if (OK != get_channel_current(DIGITAL_12V_CURRENT_REG, &pwr_12v_digital_i)) {
		return -EIO;
	}

	if (OK != get_channel_current(ANALOG_12V_CURRENT_REG, &pwr_12v_analog_i)) {
		return -EIO;
	}

	/* apply conversion and calibration values */
	pwr_brd_system_v = (MV_TO_V(pwr_brd_system_v) * SYSTEM_VOLTAGE_SCALE - _scale._bias_cal_term_system_volt) *
			   _scale._SF_cal_term_system_volt;
	pwr_brd_system_i = (MV_TO_V(pwr_brd_system_i) * SYSTEM_CURRENT_SCALE - _scale._bias_cal_term_system_amp) *
			   _scale._SF_cal_term_system_amp;

	pwr_brd_bat_v = (MV_TO_V(pwr_brd_bat_v) * BATTERY_VOLTAGE_SCALE - _scale._bias_cal_term_battery_volt) *
			_scale._SF_cal_term_battery_volt;
	pwr_brd_bat_i = (MV_TO_V(pwr_brd_bat_i) * BATTERY_CURRENT_SCALE - _scale._bias_cal_term_battery_amp) *
			_scale._SF_cal_term_battery_amp;

	pwr_5v_digital_i = (pwr_5v_digital_i - _scale._bias_cal_term_5v_digital_amp) * _scale._SF_cal_term_5v_digital_amp;
	pwr_5v_analog_i = (pwr_5v_analog_i - _scale._bias_cal_term_5v_analog_amp) * _scale._SF_cal_term_5v_analog_amp;
	pwr_12v_digital_i = (pwr_12v_digital_i - _scale._bias_cal_term_12v_digital_amp) * _scale._SF_cal_term_12v_digital_amp;
	pwr_12v_analog_i = (pwr_12v_analog_i - _scale._bias_cal_term_12v_analog_amp) * _scale._SF_cal_term_12v_analog_amp;

	/* generate a new report */
	report.timestamp = hrt_absolute_time();

	report.pwr_brd_status  	= pwr_brd_status;
	report.pwr_brd_led_status  = pwr_brd_led_status;

	report.pwr_brd_blink_reg  = pwr_brd_blink_reg;
	report.pwr_brd_led_1_pwr  = pwr_brd_led_1_pwr;
	report.pwr_brd_led_2_pwr  = pwr_brd_led_2_pwr;
	report.pwr_brd_led_3_pwr  = pwr_brd_led_3_pwr;
	report.pwr_brd_led_4_pwr  = pwr_brd_led_4_pwr;

	/* filter values */
	if (FILTERVALUES) {
		report.pwr_brd_system_volt = _filter_system_v.apply(pwr_brd_system_v);
		report.pwr_brd_battery_volt = _filter_bat_v.apply(pwr_brd_bat_v);
		report.pwr_brd_system_amp = _filter_system_i.apply(pwr_brd_system_i);
		report.pwr_brd_battery_amp = _filter_bat_i.apply(pwr_brd_bat_i);
		report.pwr_5v_analog_amp = _filter_5v_analog_i.apply(pwr_5v_analog_i);
		report.pwr_5v_digital_amp = _filter_5v_digital_i.apply(pwr_5v_digital_i);
		report.pwr_12v_analog_amp = _filter_12v_analog_i.apply(pwr_12v_analog_i);
		report.pwr_12v_digital_amp = _filter_12v_digital_i.apply(pwr_12v_digital_i);

	} else {
		report.pwr_brd_system_volt = pwr_brd_system_v;
		report.pwr_brd_battery_volt = pwr_brd_bat_v;
		report.pwr_brd_system_amp = pwr_brd_system_i;
		report.pwr_brd_battery_amp = pwr_brd_bat_i;
		report.pwr_5v_analog_amp = pwr_5v_analog_i;
		report.pwr_5v_digital_amp = pwr_5v_digital_i;
		report.pwr_12v_analog_amp = pwr_12v_analog_i;
		report.pwr_12v_digital_amp = pwr_12v_digital_i;
	}

	_px4_voliro_pb.set_error_count(perf_event_count(_comms_errors));

	_px4_voliro_pb.update(report);
#if 0
	PX4_INFO("VOLIRO_PB: %x, %x, %x, %u, %u, %u, %u, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f",
		 report.pwr_brd_status,
		 report.pwr_brd_led_status,
		 report.pwr_brd_blink_reg,
		 report.pwr_brd_led_1_pwr,
		 report.pwr_brd_led_2_pwr,
		 report.pwr_brd_led_3_pwr,
		 report.pwr_brd_led_4_pwr,
		 (double) report.pwr_brd_system_volt,
		 (double) report.pwr_brd_battery_volt,
		 (double) report.pwr_brd_system_amp,
		 (double) report.pwr_brd_battery_amp,
		 (double) report.pwr_5v_analog_amp,
		 (double) report.pwr_5v_digital_amp,
		 (double) report.pwr_12v_analog_amp,
		 (double) report.pwr_12v_digital_amp);
#endif
	perf_end(_sample_perf);

	return OK;
}
/* Perform self test */
int
VOLIRO_PB::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		//ToDo: implement self test function
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

/* Get registers value */
int
VOLIRO_PB::get_regs(uint8_t ptr, uint8_t *regs)
{
	uint8_t data[2];

	if (OK != _interface->read(ptr, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*regs  = data[1];

	return OK;
}

/* Set registers value */
int
VOLIRO_PB::set_regs(uint8_t ptr, uint8_t value)
{
	uint8_t data[2];

	if (OK != _interface->write(ptr, &value, 1)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	/* wait between transfers*/
	usleep(10);

	/* read back the reg and verify */

	if (OK != _interface->read(ptr, &data[0], 2)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (data[1] != value) {
		return -EIO;
	}

	return OK;
}

/* Set LED status */
int
VOLIRO_PB::set_LED_status(uint8_t pwr_brd_led_status)
{
	pwr_brd_led_status = (uint8_t)(pwr_brd_led_status & 0x0f);

	if (OK != set_regs(LED_STATS_REG, pwr_brd_led_status)) {
		return -EIO;
	}

	return OK;
}

/* Set LED power */
int
VOLIRO_PB::set_LED_power(uint8_t ptr, uint8_t pwr_brd_led_pwr)
{
	/* power -> scaled to 0-255 */

	if (OK != set_regs(ptr, pwr_brd_led_pwr)) {
		return -EIO;
	}

	return OK;
}

/* Set LED blink interval in sec*/
int
VOLIRO_PB::set_LED_blink_interval(uint8_t blink_interval_sec)
{
	uint8_t ptr = LED_REG;
	uint8_t led_blink_Reg;

	if (OK != get_regs(ptr, &led_blink_Reg)) {
		return -EIO;
	}

	led_blink_Reg = ((led_blink_Reg << 4) & 0xf0) | (blink_interval_sec & 0x0f);

	if (OK != set_regs(LED_REG, led_blink_Reg)) {
		return -EIO;
	}

	return OK;
}

/* Set LED mask */
int
VOLIRO_PB::set_LED_mask(uint8_t led_mask)
{
	uint8_t ptr = LED_REG;
	uint8_t led_blink_Reg;

	if (OK != get_regs(ptr, &led_blink_Reg)) {
		return -EIO;
	}

	led_blink_Reg = ((led_mask << 4) & 0xf0) | (led_blink_Reg & 0x0f);

	if (OK != set_regs(LED_REG, led_blink_Reg)) {
		return -EIO;
	}

	return OK;
}

/* Get system/bat voltage */
int
VOLIRO_PB::get_voltage(uint8_t ptr, float *voltage)
{
	uint8_t data[3];

	if (OK != _interface->read(ptr, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*voltage = ((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1])) * VOLTAGE_FULLSCALE / ADC_FULLSCALE;

	return OK;
}

/* Get channel current */
int
VOLIRO_PB::get_channel_current(uint8_t ptr, float *channel_current)
{
	uint8_t data[3];

	if (OK != _interface->read(ptr, &data[0], 3)) {
		perf_count(_comms_errors);
		return -EIO;
	}

	*channel_current = ((((float)(((uint16_t)(data[2] & 0x3) << 8) | data[1])) * VOLTAGE_FULLSCALE / ADC_FULLSCALE) -
			    ZERO_CURRENT_OFFSET_VOLTAGE) / CHANNEL_CURRENT_CONV_FACTOR;

	return OK;
}

/* Print calibration information */
int
VOLIRO_PB::print_cal_info()
{
	perf_print_counter(_sample_perf);

	PX4_INFO("offsets system voltage (%.2f)\n", (double)_scale._bias_cal_term_system_volt);
	PX4_INFO("scaling system voltage (%.2f)\n", (double)_scale._SF_cal_term_system_volt);
	PX4_INFO("offsets system current (%.2f)\n", (double)_scale._bias_cal_term_system_amp);
	PX4_INFO("scaling system current (%.2f)\n", (double)_scale._SF_cal_term_system_amp);
	PX4_INFO("offsets battery voltage (%.2f)\n", (double)_scale._bias_cal_term_battery_volt);
	PX4_INFO("scaling battery voltage (%.2f)\n", (double)_scale._SF_cal_term_battery_volt);
	PX4_INFO("offsets battery current (%.2f)\n", (double)_scale._bias_cal_term_battery_amp);
	PX4_INFO("scaling battery current (%.2f)\n", (double)_scale._SF_cal_term_battery_amp);
	PX4_INFO("offsets 5V digital voltage (%.2f)\n", (double)_scale._bias_cal_term_5v_digital_amp);
	PX4_INFO("scaling 5V digital voltage (%.2f)\n", (double)_scale._SF_cal_term_5v_digital_amp);
	PX4_INFO("offsets 5V analog voltage (%.2f)\n", (double)_scale._bias_cal_term_5v_analog_amp);
	PX4_INFO("scaling 5V analog voltage (%.2f)\n", (double)_scale._SF_cal_term_5v_analog_amp);
	PX4_INFO("offsets 12V digital voltage (%.2f)\n", (double)_scale._bias_cal_term_12v_digital_amp);
	PX4_INFO("scaling 12V digital voltage (%.2f)\n", (double)_scale._SF_cal_term_12v_digital_amp);
	PX4_INFO("offsets 12V analog voltage (%.2f)\n", (double)_scale._bias_cal_term_12v_analog_amp);
	PX4_INFO("scaling 12V analog voltage (%.2f)\n", (double)_scale._SF_cal_term_12v_analog_amp);

	return OK;
}

/* Print status */
void
VOLIRO_PB::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

/* Update parameters */
int
VOLIRO_PB::update_params()
{
	int32_t value = 0;

	param_get(param_find("LED_BLINK_INT"), &value);
	_pwr_brd_led_blink_int = ((uint8_t) value) & 0xf;

	param_get(param_find("LED_MASK"), &value);
	_pwr_brd_led_mask = ((uint8_t) value) & 0xf;

	param_get(param_find("LED_PWR_1"), &value);
	_pwr_brd_led_power_1 = (uint8_t)(value * 255 / 100);

	param_get(param_find("LED_PWR_2"), &value);
	_pwr_brd_led_power_2 = (uint8_t)(value * 255 / 100);

	param_get(param_find("LED_PWR_3"), &value);
	_pwr_brd_led_power_3 = (uint8_t)(value * 255 / 100);

	param_get(param_find("LED_PWR_4"), &value);
	_pwr_brd_led_power_4 = (uint8_t)(value * 255 / 100);

	param_get(param_find("CAL_B_SYS_VOLT"), &_scale._bias_cal_term_system_volt);
	param_get(param_find("CAL_SF_SYS_VOLT"), &_scale._SF_cal_term_system_volt);
	param_get(param_find("CAL_B_SYS_CUR"), &_scale._bias_cal_term_system_amp);
	param_get(param_find("CAL_SF_SYS_CUR"), &_scale._SF_cal_term_system_amp);

	param_get(param_find("CAL_B_BAT_VOLT"), &_scale._bias_cal_term_battery_volt);
	param_get(param_find("CAL_SF_BAT_VOLT"), &_scale._SF_cal_term_battery_volt);
	param_get(param_find("CAL_B_BAT_CUR"), &_scale._bias_cal_term_battery_amp);
	param_get(param_find("CAL_SF_BAT_CUR"), &_scale._SF_cal_term_battery_amp);

	param_get(param_find("CAL_B_5V_D_CUR"), &_scale._bias_cal_term_5v_digital_amp);
	param_get(param_find("CAL_SF_5V_D_CUR"), &_scale._SF_cal_term_5v_digital_amp);
	param_get(param_find("CAL_B_5V_A_CUR"), &_scale._bias_cal_term_5v_analog_amp);
	param_get(param_find("CAL_SF_5V_A_CUR"), &_scale._SF_cal_term_5v_analog_amp);

	param_get(param_find("CAL_B_12V_D_CUR"), &_scale._bias_cal_term_12v_digital_amp);
	param_get(param_find("CAL_SF_12V_D_CUR"), &_scale._SF_cal_term_12v_digital_amp);
	param_get(param_find("CAL_B_12V_A_CUR"), &_scale._bias_cal_term_12v_analog_amp);
	param_get(param_find("CAL_B_12V_A_CUR"), &_scale._SF_cal_term_12v_analog_amp);

	/* sensor configuration with the updated values */
	if (configure_sensor() == PX4_OK) {
		ScheduleDelayed(10_ms);

	} else {
		PX4_INFO("Fail to set VOLIRO_PB parameters");
		return -EIO;
	}

	return OK;
}
