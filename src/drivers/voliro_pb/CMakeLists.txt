############################################################################
#
#   Copyright (c) 2021 ETH Zurich - Wyss Zurich. All rights reserved.
#
#
############################################################################
px4_add_module(
	MODULE drivers__voliro_pb
	MAIN voliro_pb
	COMPILE_FLAGS
			-O0
	SRCS
		VOLIRO_PB.hpp
		VOLIRO_PB.cpp
		VOLIRO_PB_I2C.cpp
		VOLIRO_PB_main.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers_voliro_pb
		px4_work_queue
	)