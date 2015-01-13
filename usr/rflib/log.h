#ifndef __LOG_H__
#define __LOG_H__
/*
 * Copyright 2011 Freescale Semiconductor, Inc.
 *
 */

#include <stdio.h>

#ifdef DEBUG
#define DBG_INFO(...) fprintf(stderr, __VA_ARGS__)
#else
#define DBG_INFO(...)
#endif

#define DBG_ERR(...) fprintf(stderr, __VA_ARGS__)
#define DBG_PERROR(x, ...) fprintf(stderr, "" x " \
in function <%s> in line <%d>\n \
	<%m>\n", __func__, __LINE__, ##__VA_ARGS__)
#endif
