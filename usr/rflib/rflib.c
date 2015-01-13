/*
 * riflib.c
 * RF device configuration library
 *
 * Radio interface device framework of kernel exposes the combination of a
 * RF interface controller and RFIC as a RF device (eg. rf0, rf1) to user
 * space for configuration. This interface is for RF PHYs for LTE/CDMA systems.
 *
 * This library provides command line interface to:
 * 1. get information about RF device
 * 2. configure RF device.
 *
 * Author: Eyal Wurmbrand <eyalwu@freescale.com>
 *
 * Copyright 2011-2015 Freescale Semiconductor, Inc.
 *
 */

#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include "rf_if.h"
#include "log.h"
#include "rflib.h"

#define DEV_DLT_NAME_PATTERN "/dev/dltxx_y"

static char  DEV_DLT_NAME_RF[2][sizeof(DEV_DLT_NAME_PATTERN)];
static int   DEV_DLT_ID_RF[2]   = { -1, -1 };

int rfdev_get_fd(rf_handle_t rf_handle)
{
	struct rflib_rfdev_priv *priv = rf_handle->priv;

	return priv->fd;
}

int rfdev_close(rf_handle_t rf_handle)
{
	int fd;
	
	fd = rfdev_get_fd(rf_handle);
	close(fd);
	free(rf_handle);

	return 1;
}

static int get_dlt_module(int dlt_id) {

	return dlt_id >> 2;
}
static int get_dlt_channel(int dlt_id) {

	return dlt_id & 0x3;
}



int rflib_init_params(int rf0_dlt, int rf1_dlt)
{
	int i,j;
	if (rf0_dlt > 127 || rf1_dlt > 127)
	{
		DBG_ERR("Failed to init, DLT index out of bound, err %d\n", errno);
	}
	if (rf0_dlt == -1)
		rf0_dlt = DEV_DEFAULT_RF0_DLT;
	if (rf1_dlt == -1)
		rf1_dlt = DEV_DEFAULT_RF1_DLT;
	for (i = 0; i < 2; i++) {
		for ( j = 0; j < sizeof(DEV_DLT_NAME_PATTERN); j++) {
			DEV_DLT_NAME_RF[i][j] = DEV_DLT_NAME_PATTERN[j];
		}

	}

	DEV_DLT_ID_RF[0] = rf0_dlt;
	DEV_DLT_NAME_RF[0][sizeof("/dev/dlt") - 1]     = (char)('0' + (get_dlt_module(rf0_dlt)/10));
	DEV_DLT_NAME_RF[0][sizeof("/dev/dltx") - 1]    = (char)('0' + (get_dlt_module(rf0_dlt)%10));
	DEV_DLT_NAME_RF[0][sizeof("/dev/dltxx_") - 1]  = (char)('0' + (get_dlt_channel(rf0_dlt)));

	DEV_DLT_ID_RF[1] = rf1_dlt;
	DEV_DLT_NAME_RF[1][sizeof("/dev/dlt") - 1]     = (char)('0' + (get_dlt_module(rf1_dlt)/10));
	DEV_DLT_NAME_RF[1][sizeof("/dev/dltx") - 1]    = (char)('0' + (get_dlt_module(rf1_dlt)%10));
	DEV_DLT_NAME_RF[1][sizeof("/dev/dltxx_") - 1]  = (char)('0' + (get_dlt_channel(rf1_dlt)));

	return 0;
}

int rflib_init(void)
{
	return rflib_init_params(-1, -1);
}
rf_handle_t rfdev_open(const char *if_name)
{

	const char *dev_name;
	rf_handle_t rf_handle = NULL;
	struct rflib_rfdev_priv *priv;
	int dev_idx, size = 0;
	unsigned int dlt_id = 0;
	if (!if_name) {
		DBG_ERR("Failed to open, no name specified, err %d\n", errno);
		goto out;
	}
	if (!strncasecmp(if_name, "rf0", DEV_NAME_SIZE)) 
		dev_idx = 0;
	else if (!strncasecmp(if_name, "rf1", DEV_NAME_SIZE)) 
		dev_idx = 1;
	else {
		DBG_ERR("Failed to open, %s is not a valid name, err %d\n", if_name, errno);
		goto out;
	}
	
	dev_name = DEV_DLT_NAME_RF[dev_idx];
	dlt_id = DEV_DLT_ID_RF[dev_idx];

	size = sizeof(struct rf_handle) + sizeof(struct rflib_rfdev_priv);

	rf_handle = malloc(size);
	if (!rf_handle) {
		DBG_ERR("Failed to allocate memory for rf_handle, err : %d\n",
				errno);
		goto out;
	}

	memset(rf_handle, 0, size);

	rf_handle->priv = ((char *) rf_handle) + sizeof(struct rf_handle);
	priv = (struct rflib_rfdev_priv *) rf_handle->priv;
	strncpy(rf_handle->name, if_name, DEV_NAME_SIZE);

	priv->dlt_id = dlt_id;
	priv->fd = open(dev_name, O_RDWR);
	if (priv->fd < 0) {
		DBG_ERR("Failed to open %s, err %d\n",
			dev_name, errno);
		goto out;
	}

	return rf_handle;
out:
	if (rf_handle) {
		if (priv->fd > 0)
			close(priv->fd);
		free(rf_handle);
	}
	return NULL;
}


void rflib_exit(void)
{
	return;
}

int rfdev_get_devinfo(rf_handle_t rf_handle,
	struct rf_dev_info *dev_info)
{	
	struct rflib_rfdev_priv *priv = rf_handle->priv;
	dev_info->dlt_id = priv->dlt_id;

	return 0;
}
