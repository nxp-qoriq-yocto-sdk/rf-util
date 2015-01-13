#ifndef __RF_IF_H__
#define __RF_IF_H__
/*
 * rf_if.h
 * Author: Eyal Wurmbrand <eyalwu@freescale.com>
 *
 * Copyright 2011-2015 Freescale Semiconductor, Inc.
 *
 */

#include <sys/types.h>
#include <stdint.h>

#define DEV_NAME_SIZE	30

struct rf_handle {
	char name[DEV_NAME_SIZE];
	void *priv;
};


struct rf_dev_info {
	int dlt_id;
};

typedef struct rf_handle *rf_handle_t;

struct rflib_rf_init_params {
	unsigned int dlt_timer;
};

int rflib_init_params(int rf0_dlt, int rf1_dlt);
int	rflib_init(void);
rf_handle_t rfdev_open(const char *if_name);
int	rfdev_get_fd(rf_handle_t rf_handle);
int	rfdev_close(rf_handle_t rf_handle);
int	rfdev_get_devinfo(rf_handle_t rf_handle,
		struct rf_dev_info *dev_info);
void	rflib_exit(void);
#endif
