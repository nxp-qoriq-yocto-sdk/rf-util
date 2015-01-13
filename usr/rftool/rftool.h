#ifndef __RIFTOOL_H__
#define __RIFTOOL_H__
/*
 * rftool.h
 * Author: Eyal Wurmbrand <eyalwu@freescale.com>
 *
 * Copyright 2011-2014 Freescale Semiconductor, Inc.
 *
 */

#define MAX_STRING_LEN	40
#define FILE_NAME_LEN	200
#define TTI_LOG_COUNT	100
#define VERSION	"v1.1"
#include <stdint.h>
#include "rf_if.h"

#define WITH_ARG	1
#define WITHOUT_ARG	0
#define OPTIONAL_ARG	2
enum rfdev_index {
	RF0,
	RF1
};

enum rftool_cmd {
	TEST_DL_TTI,
	DUMP_DEVINFO,
	CMD_END
};

struct rfdev {
	char if_name [MAX_STRING_LEN];
	rf_handle_t if_handle;
};

struct rfdev_params {
	unsigned int dlt_id;
};

#define CMD_DL_TTI_TEST		"dl_tti_test"
#define CMD_CALIBRATE		"calibrate"
#endif
