/****************************************************************************
 *
 *   Copyright (c) 2020 Wyss Zurich - ETH Zurich Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "VOLIRO_PB.hpp"

namespace voliro_pb
{
extern device::Device *VOLIRO_PB_I2C_interface(uint8_t bus, uint32_t device, int bus_frequency);
}

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

using namespace voliro_pb;

void
VOLIRO_PB::print_usage()
{
	PRINT_MODULE_USAGE_NAME("voliro_pb", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("sensor_voliro_pb");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(VOLIRO_PB_SLAVE_ADDRESS);
	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *VOLIRO_PB::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	device::Device *interface = nullptr;

	if (iterator.busType() == BOARD_I2C_BUS) {
		interface = VOLIRO_PB_I2C_interface(iterator.bus(), cli.i2c_address, cli.bus_frequency);
	}

	if (interface == nullptr) {
		PX4_ERR("failed creating interface for bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	VOLIRO_PB *dev = new VOLIRO_PB(iterator.configuredBusOption(), iterator.bus(), interface);

	if (dev == nullptr) {
		delete interface;
		return nullptr;
	}

	if (OK != dev->init()) {
		delete dev;
		return nullptr;
	}

	return dev;
}

extern "C" int voliro_pb_main(int argc, char *argv[])
{
	using ThisDriver = VOLIRO_PB;
	BusCLIArguments cli{true, false};
	cli.i2c_address = VOLIRO_PB_SLAVE_ADDRESS;
	cli.default_i2c_frequency = VOLIRO_PB_BUS_SPEED;
	cli.support_keep_running = true;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_PWR_DEVTYPE_VOLIRO_PB);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
