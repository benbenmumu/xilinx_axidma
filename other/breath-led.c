/*  breath-led.c - The simplest kernel module.

* Copyright (C) 2013 - 2016 Xilinx, Inc
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License along
*   with this program. If not, see <http://www.gnu.org/licenses/>.

*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/bitops.h>
#include <linux/dmapool.h>
#include <linux/dma/xilinx_dma.h>
#include <linux/iopoll.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include "breath-led.h"

static void __iomem *base_addr; // to store driver specific base address needed for mmu to translate virtual address to physical address in our FPGA design

/* IO accessors */
static inline u32 reg_read(u32 reg)
{
    return ioread32(base_addr + reg);
}

static inline void reg_write(u32 reg, u32 value)
{
    iowrite32(value, base_addr + reg);
}

/* Register operation */
static inline void breath_led_enable(void)
{
    reg_write(BREATH_LED_ENABLE_ADDR, 1);
}

static inline void breath_led_disable(void)
{
    reg_write(BREATH_LED_ENABLE_ADDR, 0);
}

static inline void breath_led_set_freq(u32 freq)
{
    reg_write(BREATH_LED_FREQ_ADDR, freq);
}

static inline u32 breath_led_get_freq(void)
{
    return reg_read(BREATH_LED_FREQ_ADDR);
}

static struct breath_led_driver_api breath_led_driver_api_inst;
static struct breath_led_driver_api *breath_led_api = &breath_led_driver_api_inst;
EXPORT_SYMBOL(breath_led_api);

/* device struct */
struct breath_led_dev {
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
};

struct breath_led_dev breath_led;

static int breath_led_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t breath_led_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
    int ret, enable;
    char readin[20];

    ret = copy_from_user(readin, buf, cnt);
    readin[cnt] = 0;
	ret = kstrtoint(readin, 10, &enable);	// 得到应用层传递过来的数据
	if(0 > ret) {
		printk(KERN_ERR "breath_led: Failed to copy data from user buffer\r\n");
		return -EFAULT;
	}

    printk(KERN_ERR "breath_led: set enable %d\r\n", enable);

	if (enable == 0)
		breath_led_api->breath_led_disable();		// 如果传递过来的数据是0则关闭led
	else if (enable == 1)
        breath_led_api->breath_led_enable();		// 如果传递过来的数据是1则点亮led

	return cnt;
}

static struct file_operations breath_led_ops = {
    .owner = THIS_MODULE,
    .open = breath_led_open,
	.write = breath_led_write,
};

static ssize_t att_store(struct device *dev,
                        struct device_attribute *attr,
                        const char *buf, size_t count)
{
    u32 ret, freq;

    printk(KERN_ERR "breath_led: attr: %s\n", attr->attr.name);
	ret = kstrtouint(buf, 10, &freq);	// 得到应用层传递过来的数据
	if(0 > ret) {
		printk(KERN_ERR "breath_led: Failed to copy data from user buffer\r\n");
		return -EFAULT;
	}
    breath_led_api->breath_led_set_freq(freq);
    return count;
}

static ssize_t att_show(struct device *dev,
                        struct device_attribute *attr,
                        char *buf)
{
    printk(KERN_ERR "breath_led: attr: %s\n", attr->attr.name);
    u32 freq = breath_led_api->breath_led_get_freq();
    return sprintf(buf, "0x%x\n", freq);
}

static DEVICE_ATTR(freq, 0664, att_show, att_store);
static DEVICE_ATTR(enable, 0664, att_show, att_store);

static const struct of_device_id dev_of_ids[] = {
    {
        .compatible = "xlnx,breath-led-ip-1.0",
    },
    {}};
MODULE_DEVICE_TABLE(of, dev_of_ids);

static inline u32 hw_init(u32 enable, u32 freq)
{
    int ret = 0;

    printk("%s breath_led enable %d\n", breath_led_compatible_str, enable);

    breath_led_api->breath_led_enable();
    breath_led_api->breath_led_set_freq(freq);

    return (ret);
}

static int dev_probe(struct platform_device *pdev)
{
    struct device_node *np = pdev->dev.of_node;
    struct resource *io;
    int ret = 1;

    printk("\n");

    if (np)
    {
        const struct of_device_id *match;

        match = of_match_node(dev_of_ids, np);
        if (match)
        {
            printk("%s dev_probe match!\n", breath_led_compatible_str);
            ret = 0;
        }
    }

    if (ret)
        return ret;

    breath_led_api->hw_init = hw_init;

    breath_led_api->reg_read = reg_read;
    breath_led_api->reg_write = reg_write;
    breath_led_api->breath_led_enable = breath_led_enable;
    breath_led_api->breath_led_disable = breath_led_disable;
    breath_led_api->breath_led_set_freq = breath_led_set_freq;
    breath_led_api->breath_led_get_freq = breath_led_get_freq;

    /* Request and map I/O memory */
    io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    base_addr = devm_ioremap_resource(&pdev->dev, io);
    if (IS_ERR(base_addr))
        return PTR_ERR(base_addr);

    breath_led_api->io_start = io->start;
    breath_led_api->base_addr = (u32)base_addr;

    printk("%s dev_probe succeed!\n", breath_led_compatible_str);

    ret = hw_init(1, 0x80000032);

    /* 初始化cdev */
	ret = alloc_chrdev_region(&breath_led.devid, 0, 1, "breath_led");
	if (ret)
		goto out1;

	breath_led.cdev.owner = THIS_MODULE;
	cdev_init(&breath_led.cdev, &breath_led_ops);

	/* 添加cdev */
	ret = cdev_add(&breath_led.cdev, breath_led.devid, 1);
	if (ret)
		goto out2;

	/* 创建类class */
	breath_led.class = class_create(THIS_MODULE, "breath_led");
	if (IS_ERR(breath_led.class)) {
		ret = PTR_ERR(breath_led.class);
		goto out3;
	}

	/* 创建设备 */
	breath_led.device = device_create(breath_led.class, &pdev->dev,
				breath_led.devid, NULL, "breath_led");
	if (IS_ERR(breath_led.device)) {
		ret = PTR_ERR(breath_led.device);
		goto out4;
	}

    /* create attribute */
    ret = device_create_file(&pdev->dev, &dev_attr_freq);
    if (ret)
    {
        printk(KERN_ERR "breath_led: Failed to create file\r\n");
        goto out5;
    }

	return 0;

out5:
    device_destroy(breath_led.class, breath_led.devid);

out4:
	class_destroy(breath_led.class);

out3:
	cdev_del(&breath_led.cdev);

out2:
	unregister_chrdev_region(breath_led.devid, 1);

out1:
    return ret;
}

static int dev_remove(struct platform_device *pdev)
{
    printk("\n");
    breath_led_api->breath_led_disable();

    /* remove the attribute file */
    device_remove_file(&pdev->dev, &dev_attr_freq);

    /* 注销设备 */
	device_destroy(breath_led.class, breath_led.devid);

	/* 注销类 */
	class_destroy(breath_led.class);

	/* 删除cdev */
	cdev_del(&breath_led.cdev);

	/* 注销设备号 */
	unregister_chrdev_region(breath_led.devid, 1);

    printk("%s dev_remove succeed!\n", breath_led_compatible_str);
    return 0;
}

static struct platform_driver dev_driver = {
    .driver = {
        .name = "breath-led",
        .owner = THIS_MODULE,
        .of_match_table = dev_of_ids,
    },
    .probe = dev_probe,
    .remove = dev_remove,
};

module_platform_driver(dev_driver);

MODULE_AUTHOR("mumu");
MODULE_DESCRIPTION("xlnx,breath-led-ip-1.0");
MODULE_LICENSE("GPL v2");
