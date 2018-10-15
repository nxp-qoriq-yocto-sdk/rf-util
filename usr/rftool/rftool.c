/*
 * rftool.c
 * RF device configuration utility
 *
 * Radio interface device framework of kernel exposes the combination of a
 * RF interface controller and RFIC as a RF device (eg. rf0, rf1) to user
 * space for configuration. This interface is for RF PHYs for LTE/CDMA systems.
 *
 * This utility provides command line interface to:
 * 1. get information about RF deivce
 * 2. configure RF device.
 *
 * Author: Eyal Wurmbrand <eyalwu@freescale.com>
 *
 * Copyright 2011-2015 Freescale Semiconductor, Inc.
 *
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include "log.h"
#include "rftool.h"
#include "utilities.h"
#include "rflib.h"

static int show_dev(struct rfdev *dev);
void show_usage();

uint8_t verbocity;

int show_all_devs(void)
{
	return 0;
}


int get_rfdev_index(char *rfdev_name)
{
	int rfdev_index = 0;

	if (!strncasecmp(rfdev_name, "rf0", MAX_STRING_LEN))
		rfdev_index = RF0;
	else if (!strncasecmp(rfdev_name, "rf1", MAX_STRING_LEN))
		rfdev_index = RF1;

	return rfdev_index;
}

static int show_dev(struct rfdev *dev)
{
	int rc = 0;
	struct rf_dev_info dev_info;

	memset(&dev_info, 0, sizeof(struct rf_dev_info));
	rc = rfdev_get_devinfo(dev->if_handle, &dev_info);
	if (rc)
		return rc;
	printf("	%s is using DLT #%d\n", dev->if_name, dev_info.dlt_id);

	return 0;
}


int test_dl_tti(struct rfdev *dev)
{
	int fd, rc = 0;

#ifndef NON_BLOCKING_MODE
	struct timeval tv[TTI_LOG_COUNT];
	unsigned int tti_count[TTI_LOG_COUNT], i = 0;
#else
	fd_set f_descs;
	struct timeval timeout = {0, 0};
	dlt_read_t dlt_read = {0};
#endif

	fd = rfdev_get_fd(dev->if_handle);

	if (!fd)
	   return -1;

#ifndef NON_BLOCKING_MODE
	memset(&tv[0], 0, sizeof(tv));
	memset(&tti_count[0], 0, sizeof(tti_count));

	while (i < TTI_LOG_COUNT) {
		rc = read(fd, &tti_count[i], sizeof(unsigned int));
		if (rc < 0) {
			DBG_ERR("TTI Not coming, err %d\n", errno);
			goto out;
		}
		gettimeofday(&tv[i], NULL);
		i++;
	}

	for (i = 0; i < TTI_LOG_COUNT; i++) {
		printf("[%d] tti 0x%0x, time %d:%d\n", i, tti_count[i],
			(int) tv[i].tv_sec, (int) tv[i].tv_usec);
	}
out:
	return rc;
#else
	timeout.tv_sec = 5;	/* Add Value */
	timeout.tv_usec = 0;

	FD_ZERO(&f_descs);
	FD_SET(fd, &f_descs);

	rc = select(fd + 1, &f_descs, NULL, NULL, &timeout);
	if (rc == -1)
		perror("select()");
	else if (rc) {
		printf("Data is available now\n");
		rc = read(fd, &dlt_read, sizeof(dlt_read_t));

		if (rc < sizeof(dlt_read_t)) {
			printf("Failed to read from RF file descriptor. return = %d.", rc);
			exit(-1);
		} else {
			printf("Count = %d reg_cntr = %d\n",
					dlt_read.count, dlt_read.reg_cntr);
			return rc;
		}
	} else
		printf("no data within 5 Seconds\n");
#endif

return -1;
}

enum rftool_cmd get_cmd(char *cmd_string)
{
	if (!cmd_string)
		return -1;

	if (!strncasecmp(cmd_string, CMD_DL_TTI_TEST, MAX_STRING_LEN))
		return TEST_DL_TTI;

	return -1;
}

void show_usage()
{
	printf("\nrftool version %s\n", VERSION);
	printf("rftool -i rf_interface_name -c <optional commands> -f<optional dlt first rf> -s<optional dlt second rf>\n");

	printf("commands:\n");
	printf("	  : %s\n", CMD_DL_TTI_TEST);

}

int main(int argc, char *argv[])
{
	int opt, rc = 0, args = 0;
	struct rfdev dev;
	struct rf_dev_info dev_info;
	enum rftool_cmd cmd = CMD_END;
	int first_rf = -1, second_rf = -1;

	while ((opt = getopt(argc, argv, "i:c:f:s:")) != -1) {
		switch (opt) {
		case 'c':
			cmd = get_cmd(optarg);
			args++;
			break;
		case 'i':
			cmd = DUMP_DEVINFO;
			strncpy(&dev.if_name[0], optarg, DEV_NAME_SIZE);
			args++;
			break;
		case 'f':
			first_rf = atoi(optarg);
			args++;
			break;
		case 's':
			second_rf = atoi(optarg);
			args++;
			break;
		default:
			rc = EINVAL;
		}
	}
	if (!args) {
		rc = show_all_devs();
		return EINVAL;
	}

	if (!dev.if_name[0]) {
		DBG_ERR("No Interface name!\n");
		return EINVAL;
	}

	if (rc) {
		show_usage();
		goto out;
	}

	rc = rflib_init_params(first_rf, second_rf);

	if (rc)
		goto out;

	dev.if_handle = rfdev_open(dev.if_name);
	if (!dev.if_handle) {
		DBG_ERR("Failed to open rfdev %s\n", dev.if_name);
		goto out;
	}

	rc = rfdev_get_devinfo(dev.if_handle, &dev_info);
	if (rc < 0) {
		DBG_ERR("unable to obtain devinfo");
		goto out;
	}

	switch (cmd) {

	case DUMP_DEVINFO:
		rc = show_dev(&dev);
		break;
	case TEST_DL_TTI:

		rc = test_dl_tti(&dev);
		break;
	default:
		show_usage();
		break;
	}

out:
	if (dev.if_handle)
		rfdev_close(dev.if_handle);
	rflib_exit();
	return rc;
}
