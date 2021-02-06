uint64 timestamp          		# time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          		# unique device ID for the sensor that does not change between power cycles
uint32 error_count

uint8 pwr_brd_status			# power boards status register
uint8 pwr_brd_led_status		# power boards power leds status
uint8 pwr_brd_blink_reg			# power boards blink register, defines number of blinks and interval
uint8 pwr_brd_led_1_pwr			# power boards power led 1 intensity in percentage
uint8 pwr_brd_led_2_pwr			# power boards power led 2 intensity in percentage
uint8 pwr_brd_led_3_pwr			# power boards power led 3 intensity in percentage
uint8 pwr_brd_led_4_pwr			# power boards power led 4 intensity in percentage
float32 pwr_brd_system_volt		# power boards system voltage in volts
float32 pwr_brd_battery_volt	# power boards battery voltage in volts
float32 pwr_brd_system_amp		# power boards system current in volts
float32 pwr_brd_battery_amp		# power boards battery current in volts
float32 pwr_5v_analog_amp		# power boards 5v analog channel current in amperes
float32 pwr_5v_digital_amp		# power boards 5v digital channel current in amperes
float32 pwr_12v_analog_amp		# power boards 12v analog channel current in amperes
float32 pwr_12v_digital_amp		# power boards 12v digital channel current in amperes