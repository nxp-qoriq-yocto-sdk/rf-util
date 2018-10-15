#ifndef __RFLIB_H__
#define __RFLIB_H__

/* #include "fsl_types.h"
 * #include "fsl_usmmgr.h"
 *
 * rflib.h
 * Author: Eyal Wurmbrand <eyalwu@freescale.com>
 *
 * Copyright 2011-2015 Freescale Semiconductor, Inc.
 *
 */

#define DEV_DEFAULT_RF0_DLT 64
#define DEV_DEFAULT_RF1_DLT 69

struct rflib_rfdev_priv {
	int fd;
	int dlt_id;
};

typedef struct _dlt_read_t {
	uint32_t count;    /* RF TTI counter/id */
	uint32_t reg_cntr; /* DLT COUNTER snapshot - for debugging purposes */
} dlt_read_t;

#define PAINT_BUF_MAGIC	0xA5

#define RF_MMAPED_ANT_BUF_SIZE	((16 * 1024 * 1024) - 64)

/* This macro is added for Non-Blocking mode */
#define NON_BLOCKING_MODE	/* Remove this for blocking mode */

#endif

