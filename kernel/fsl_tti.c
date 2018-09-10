/*
 * TTI (Timer Tick Indication) Module
 *
 * Copyright 2018 NXP.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale/NXP Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale/NXP Semiconductor "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale/NXP Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Quad timers 16-31 are connected to the Power Architecture MPIC
 * Each quad timer module contains four identical timer
 * groups (32-bit) that serve as Timer Tick Indication notification
 * which is needed by L2, WWB applications.
 * This module receives Interrupts from Device level Quad timers
 * and notifies the processes waiting for it. This does not program
 * the Timers as such.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/poll.h>

#if 0
#define TIMER_DEBUG /* Debug using Kernel timer, No HW timer interrupt */
#endif

#if 0
#ifdef dev_dbg
#undef dev_dbg
#define dev_dbg(dev, format, arg ...) printk(format, ## arg)
#endif
#endif

#define NUM_OF_HW_TIMERS_PER_MODULE 4
#define NUM_OF_HW_TIMERS_MODULE     16
#define TTI_MAX_DEVICES	(NUM_OF_HW_TIMERS_MODULE * NUM_OF_HW_TIMERS_PER_MODULE)

#define TIMER_COMPARE_FLAG_BIT (1 << 15)
#define TIMER_PAYLOAD_READY_BIT 0

struct dlt_read_t {
	uint32_t count;    /* RF TTI counter/id */
	uint32_t reg_cntr; /* DLT COUNTER snapshot - for debugging purposes */
};

/* Represent timer in a timer Module */
struct tti_timer {
	struct dlt_read_t          dlt_read;
	void __iomem              *tmr_cntr;
	void __iomem              *tmr_sctl;
	wait_queue_head_t          wait_q;
	volatile unsigned long int ready;
	atomic_t                   ref;
};

/* Represent timer Module */
struct tti_timer_mod {
	struct tti_timer  timer[NUM_OF_HW_TIMERS_PER_MODULE];
	uint32_t          virq;
	atomic_t          irq_ref;
	unsigned char     name[10];
#ifdef TIMER_DEBUG
	struct timer_list my_timer; /* Testing without interrupts*/
#endif
};

struct tti_timer_mod timer_module[NUM_OF_HW_TIMERS_MODULE];

static dev_t         tti_dev;       /* first device number */
static struct cdev   tti_cdev;      /* character device structure */
static struct class *tti_class_dev; /* device class */

struct device *gdev;

static inline int tti_get_timer_mod(int minor, struct tti_timer_mod **timer_mod)
{
	*timer_mod = &timer_module[minor / NUM_OF_HW_TIMERS_PER_MODULE];
	return minor / NUM_OF_HW_TIMERS_PER_MODULE;
}

static inline int tti_get_timer(int minor, struct tti_timer **timer)
{
	*timer = &timer_module[minor / NUM_OF_HW_TIMERS_PER_MODULE].timer[minor % NUM_OF_HW_TIMERS_PER_MODULE];
	return minor % NUM_OF_HW_TIMERS_PER_MODULE;
}

static irqreturn_t tti_interrupt_handler(int irq, void *data)
{
	uint32_t sct;
	int timer;
	struct tti_timer_mod *tmr_mod = (struct tti_timer_mod *)data;

	for (timer = 0; timer < NUM_OF_HW_TIMERS_PER_MODULE; timer++) {
		sct = ioread32be(tmr_mod->timer[timer].tmr_sctl);

		if (sct & TIMER_COMPARE_FLAG_BIT) {
			iowrite32be(sct & ~TIMER_COMPARE_FLAG_BIT,
					tmr_mod->timer[timer].tmr_sctl);

		if (atomic_read(&tmr_mod->timer[timer].ref)) {
			tmr_mod->timer[timer].dlt_read.count++;
			tmr_mod->timer[timer].dlt_read.reg_cntr =
				ioread32be(tmr_mod->timer[timer].tmr_cntr);

			set_bit(TIMER_PAYLOAD_READY_BIT,
				&tmr_mod->timer[timer].ready);
			wake_up_interruptible(&tmr_mod->timer[timer].wait_q);
			}
		}
	}

	return IRQ_HANDLED;
}

#ifdef TIMER_DEBUG

static void my_timer_func(unsigned long ptr)
{
	int count;
	struct tti_timer_mod *tmr_mod = (struct tti_timer_mod *)ptr;

	dev_dbg(gdev, "Timer Expire -> %d\n", tmr_mod->virq);

	for (count = 0; count < NUM_OF_HW_TIMERS_PER_MODULE; count++) {
		if (atomic_read(&tmr_mod->timer[count].ref)) {
			tmr_mod->timer[count].dlt_read.count++;
			dev_dbg(gdev, "tmr_sctl = %p\n",
				tmr_mod->timer[count].tmr_sctl);
			dev_dbg(gdev, "read tmp_sctl = %x\n",
				ioread32be(tmr_mod->timer[count].tmr_sctl));
			swait_wake_all(&tmr_mod->timer[count].wait_q);
		}
	}

	tmr_mod->my_timer.expires = jiffies + 100;
	add_timer(&tmr_mod->my_timer);
}

static int set_my_timer(struct tti_timer_mod *timer_mod)
{
	dev_dbg(gdev, "set_my_timer for = %d\n", timer_mod->virq);

	init_timer(&timer_mod->my_timer);

	timer_mod->my_timer.function = my_timer_func;
	timer_mod->my_timer.data     = (unsigned long)timer_mod;
	timer_mod->my_timer.expires  = jiffies + 100;

	add_timer(&timer_mod->my_timer);

	return 0;
}

static int del_my_timer(struct tti_timer_mod *timer_mod)
{
	dev_dbg(gdev, "del_my_timer for = %d\n", timer_mod->virq);
	del_timer(&timer_mod->my_timer);
	return 0;
}

#endif

static int dlt_open(struct inode *inode, struct file *filep)
{
	int minor;
	int tmr_mod, tmr;
	int ret;

	struct tti_timer     *timer     = NULL;
	struct tti_timer_mod *timer_mod = NULL;

	minor = iminor(inode);

	dev_dbg(gdev, "dlt_open with minor = %d\n", minor);

	tmr     = tti_get_timer(minor, &timer);
	tmr_mod = tti_get_timer_mod(minor, &timer_mod);

	filep->private_data = timer;

	if (!atomic_read(&timer_mod->irq_ref)) {
		sprintf(timer_mod->name, "tti%d", tmr_mod);
		ret = request_irq(timer_mod->virq, tti_interrupt_handler,
					0, timer_mod->name, timer_mod);

		if (ret) {
			dev_err(gdev, "Couldn't request interrupt\n");
			return -1;
		}

		atomic_inc(&timer_mod->irq_ref);
#ifdef TIMER_DEBUG
		set_my_timer(timer_mod);
#endif
		dev_dbg(gdev, "interrupt request number %d with %s\n",
					timer_mod->virq, timer_mod->name);
	}

	if (!atomic_read(&timer->ref))
		init_waitqueue_head(&timer->wait_q);

	atomic_inc(&timer->ref);

	return 0;
}

static ssize_t dlt_read(struct file *filep, char __user *buf,
				size_t nbytes, loff_t *ppos)
{
	ssize_t ret;

	DECLARE_WAITQUEUE(wait, current);

	struct tti_timer *timer = filep->private_data;

	if (nbytes != sizeof(struct dlt_read_t))
		return -EINVAL;

	add_wait_queue(&timer->wait_q, &wait);

	do {
		__set_current_state(TASK_INTERRUPTIBLE);

		if (test_and_clear_bit(TIMER_PAYLOAD_READY_BIT, &timer->ready))
			break;

		if (filep->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto out;
			}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			goto out;
			}

		schedule();
	} while (1);

	ret = put_user(timer->dlt_read, (struct dlt_read_t __user *)buf)
				?: sizeof(timer->dlt_read);

out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&timer->wait_q, &wait);

	return ret;
}

static unsigned int dlt_poll(struct file *filep, poll_table *wait)
{
	struct tti_timer *timer;

	timer = filep->private_data;

	poll_wait(filep, &timer->wait_q, wait);

	if (test_bit(TIMER_PAYLOAD_READY_BIT, &timer->ready))
		return POLLIN | POLLRDNORM;

	/* obviously, as soon as the first process does a dlt_read(),
	 * this information becomes a lie for all the other processes
	 * that polled */

	return 0;
}

static int dlt_release(struct inode *inode, struct file *filep)
{
	struct tti_timer *timer;

	timer = filep->private_data;
	atomic_dec(&timer->ref);

	return 0;
}

static const struct file_operations dlt_fops = {
	.owner   = THIS_MODULE,
	.open    = dlt_open,
	.poll    = dlt_poll,
	.read    = dlt_read,
	.release = dlt_release
};

/* Update status control resister address in timer struct
 * This will reduce computation in IRQ handler */
static int tti_map_mem(struct platform_device *pdev)
{
	int tmr_mod, tmr;

	void __iomem *tmr_base;
	void __iomem *cnt_regs;
	void __iomem *sct_regs;

	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		dev_err(gdev, "missing memory resource\n");
		return -ENODEV;
	}

	dev_dbg(gdev, "res->start = %x\n", (unsigned int)res->start);

	tmr_base = devm_ioremap_resource(gdev, res);
	dev_dbg(gdev, "map regs address is %p\n", tmr_base);

	if (IS_ERR(tmr_base))
		return PTR_ERR(tmr_base);

	cnt_regs = tmr_base + 0x14; /* OFFSET of CNT of 1st timer in Module */
	sct_regs = tmr_base + 0x1c; /* OFFSET of SCT of 1st timer in Module */

	for (tmr_mod = 0; tmr_mod < NUM_OF_HW_TIMERS_MODULE; tmr_mod++) {
		for (tmr = 0; tmr < NUM_OF_HW_TIMERS_PER_MODULE; tmr++) {
			timer_module[tmr_mod].timer[tmr].tmr_cntr = cnt_regs;
			timer_module[tmr_mod].timer[tmr].tmr_sctl = sct_regs;

			cnt_regs += 0x40; /* OFFSET of CNT */
			sct_regs += 0x40; /* OFFSET of SCT */

			atomic_set(&timer_module[tmr_mod].timer[tmr].ref, 0);
	}

	tmr_base += 0x400; /* OFFSET of DLT Module */

	cnt_regs = tmr_base + 0x14;
	sct_regs = tmr_base + 0x1c;
	}

	return 0;
}

/* Read IRQ number from device tree and update the timer module structure.
 * The request IRQ will be happen at open time */
static int tti_map_irq(struct platform_device *pdev)
{
	int tmr_mod;

	for (tmr_mod = 0; tmr_mod < NUM_OF_HW_TIMERS_MODULE; tmr_mod++) {
		timer_module[tmr_mod].virq =
			irq_of_parse_and_map(pdev->dev.of_node, tmr_mod);

		if (timer_module[tmr_mod].virq == NO_IRQ) {
			dev_err(gdev, "Error mapping IRQ!\n");
			return -EINVAL;
		}

		dev_dbg(gdev, "Map Interrupt %d (0x%x)\n",
			timer_module[tmr_mod].virq,
			timer_module[tmr_mod].virq);
	}

	return 0;
}

static int tti_free_irq(void)
{
	int tmr_mod;

	for (tmr_mod = 0; tmr_mod < NUM_OF_HW_TIMERS_MODULE; tmr_mod++) {
		if (atomic_read(&timer_module[tmr_mod].irq_ref)) {
#ifdef TIMER_DEBUG
			del_my_timer(&timer_module[timer_mod_nu]);
#endif
			free_irq(timer_module[tmr_mod].virq,
					&timer_module[tmr_mod]);
			dev_info(gdev, "Free Interrupt %d(0x%x)\n",
					timer_module[tmr_mod].virq,
						timer_module[tmr_mod].virq);
		}
	}

	return 0;
}

/* Create Device file /dev/dltxx_y */
static int tti_device_create(struct class *class, dev_t devt)
{
	struct device *dev;
	int tmr_mod, tmr;
	unsigned char name[15];

	for (tmr_mod = 0; tmr_mod < NUM_OF_HW_TIMERS_MODULE; tmr_mod++) {
		for (tmr = 0; tmr < NUM_OF_HW_TIMERS_PER_MODULE; tmr++) {
			sprintf(name, "dlt%d_%d", tmr_mod + 16, tmr);
			dev = device_create(class, NULL, MKDEV(MAJOR(devt),
						tmr_mod * 4 + tmr), NULL, name);

			if (IS_ERR(dev))
				return PTR_ERR(dev);
		}
	}

	return 0;
}

static int tti_device_destroy(struct class *class, dev_t devt)
{
	int tmr_mod, tmr;
	unsigned char name[15];

	for (tmr_mod = 0; tmr_mod < NUM_OF_HW_TIMERS_MODULE; tmr_mod++) {
		for (tmr = 0; tmr < NUM_OF_HW_TIMERS_PER_MODULE; tmr++) {
			sprintf(name, "dlt%d_%d", tmr_mod + 17, tmr);
			device_destroy(class, MKDEV(MAJOR(devt),
						tmr_mod * 4 + tmr));
		}
	}

	return 0;
}

static int tti_probe(struct platform_device *pdev)
{
	int ret;

	gdev = &pdev->dev;

	ret = tti_map_irq(pdev);

	if (ret)
		return ret;

	ret = tti_map_mem(pdev);

	if (ret)
		return ret;

	tti_class_dev = class_create(THIS_MODULE, "tti_dev");

	if (IS_ERR(tti_class_dev)) {
		ret = PTR_ERR(tti_class_dev);
		goto err_class;
	}

	dev_dbg(gdev, "class_create Done\n");

	ret = alloc_chrdev_region(&tti_dev, 0, TTI_MAX_DEVICES, "tti_drv");

	if (ret) {
		dev_err(gdev, "Device Registration failed\n");
		goto err_chrdev;
	}

	cdev_init(&tti_cdev, &dlt_fops);

	ret = cdev_add(&tti_cdev, tti_dev, TTI_MAX_DEVICES);

	if (ret < 0) {
		dev_err(gdev, "add char device fail\n");
		goto err_cdev;
	}

	dev_dbg(gdev, "cdev_add done\n");

	ret = tti_device_create(tti_class_dev, tti_dev);

	if (ret < 0) {
		dev_err(gdev, "device creation fail\n");
		goto err_dev;
	}

	dev_dbg(gdev, "tti_probe Done\n");

	return 0;

err_dev:
	tti_device_destroy(tti_class_dev, tti_dev);

err_cdev:
	unregister_chrdev_region(tti_dev, TTI_MAX_DEVICES);

err_chrdev:
	class_destroy(tti_class_dev);
	tti_class_dev = NULL;

err_class:
	return ret;
}

static int tti_remove(struct platform_device *pdev)
{
	tti_free_irq();
	tti_device_destroy(tti_class_dev, tti_dev);
	unregister_chrdev_region(tti_dev, TTI_MAX_DEVICES);
	class_destroy(tti_class_dev);

	return 0;
}

static const struct of_device_id of_tti_match[] = {
	{.compatible = "fsl,quad-timer"},
	{},
};

MODULE_DEVICE_TABLE(of, of_tti_match);

static struct platform_driver tti_driver = {
	.probe  = tti_probe,
	.remove = tti_remove,
	.driver = {
		.name = "tti",
		.of_match_table = of_match_ptr(of_tti_match),
	},
};

module_platform_driver(tti_driver);

MODULE_AUTHOR("suresh.gupta@freescale.com");
MODULE_DESCRIPTION("TTI Timer Manager driver");
MODULE_LICENSE("GPL v2");
