/*
 * XILINX AXI DMA Engine test module
 *
 * Copyright (C) 2010 Xilinx, Inc. All rights reserved.
 *
 * Based on Atmel DMA Test Client
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched/task.h>
#include <linux/dma/xilinx_dma.h>

static unsigned int test_buf_size = 5;
module_param(test_buf_size, uint, S_IRUGO);
MODULE_PARM_DESC(test_buf_size, "Size of the memcpy test buffer");

static unsigned int iterations = 32;
module_param(iterations, uint, S_IRUGO);
MODULE_PARM_DESC(iterations, "Iterations before stopping test (default: infinite)");

/*
 * Initialization patterns. All bytes in the source buffer has bit 7
 * set, all bytes in the destination buffer has bit 7 cleared.
 *
 * Bit 6 is set for all bytes which are to be copied by the DMA
 * engine. Bit 5 is set for all bytes which are to be overwritten by
 * the DMA engine.
 *
 * The remaining bits are the inverse of a counter which increments by
 * one for each byte address.
 */
#define PATTERN_SRC 0x80
#define PATTERN_DST 0x00
#define PATTERN_COPY 0x40
#define PATTERN_OVERWRITE 0x20
#define PATTERN_COUNT_MASK 0x1f

struct dma_chan *chan, *rx_chan;

static unsigned long dmatest_random(void)
{
    unsigned long buf;

    get_random_bytes(&buf, sizeof(buf));
    return buf;
}

static void dmatest_init_srcs(u8 **bufs, unsigned int start, unsigned int len)
{
    unsigned int i;
    u8 *buf;

    for (; (buf = *bufs); bufs++)
    {
        for (i = 0; i < start; i++)
            buf[i] = PATTERN_SRC | (~i & PATTERN_COUNT_MASK);
        for (; i < start + len; i++)
            buf[i] = PATTERN_SRC | PATTERN_COPY | (~i & PATTERN_COUNT_MASK);
        for (; i < test_buf_size; i++)
            buf[i] = PATTERN_SRC | (~i & PATTERN_COUNT_MASK);
        buf++;
    }
}

static void dmatest_init_dsts(u8 **bufs, unsigned int start, unsigned int len)
{
    unsigned int i;
    u8 *buf;

    for (; (buf = *bufs); bufs++)
    {
        for (i = 0; i < start; i++)
            buf[i] = PATTERN_DST | (~i & PATTERN_COUNT_MASK);
        for (; i < start + len; i++)
            buf[i] = PATTERN_DST | PATTERN_OVERWRITE | (~i & PATTERN_COUNT_MASK);
        for (; i < test_buf_size; i++)
            buf[i] = PATTERN_DST | (~i & PATTERN_COUNT_MASK);
    }
}

static void dmatest_mismatch(u8 actual, u8 pattern, unsigned int index,
                             unsigned int counter, bool is_srcbuf)
{
    u8 diff = actual ^ pattern;
    u8 expected = pattern | (~counter & PATTERN_COUNT_MASK);
    const char *thread_name = current->comm;

    if (is_srcbuf)
        pr_warn(
            "%s: srcbuf[0x%x] overwritten! Expected %02x, got %02x\n",
            thread_name, index, expected, actual);
    else if ((pattern & PATTERN_COPY) && (diff & (PATTERN_COPY | PATTERN_OVERWRITE)))
        pr_warn(
            "%s: dstbuf[0x%x] not copied! Expected %02x, got %02x\n",
            thread_name, index, expected, actual);
    else if (diff & PATTERN_SRC)
        pr_warn(
            "%s: dstbuf[0x%x] was copied! Expected %02x, got %02x\n",
            thread_name, index, expected, actual);
    else
        pr_warn(
            "%s: dstbuf[0x%x] mismatch! Expected %02x, got %02x\n",
            thread_name, index, expected, actual);
}

static unsigned int dmatest_verify(u8 **bufs, unsigned int start,
                                   unsigned int end, unsigned int counter, u8 pattern,
                                   bool is_srcbuf)
{
    unsigned int i;
    unsigned int error_count = 0;
    u8 actual;
    u8 expected;
    u8 *buf;
    unsigned int counter_orig = counter;

    for (; (buf = *bufs); bufs++)
    {
        counter = counter_orig;
        for (i = start; i < end; i++)
        {
            actual = buf[i];
            expected = pattern | (~counter & PATTERN_COUNT_MASK);
            if (actual != expected)
            {
                if (error_count < 32)
                    dmatest_mismatch(actual, pattern, i,
                                     counter, is_srcbuf);
                error_count++;
            }
            counter++;
        }
    }

    if (error_count > 32)
        pr_warn("%s: %u errors suppressed\n",
                current->comm, error_count - 32);

    return error_count;
}

static void dmatest_slave_tx_callback(void *completion)
{
    complete(completion);
}

static void dmatest_slave_rx_callback(void *completion)
{
    complete(completion);
}

/* Function for slave transfers
 * Each thread requires 2 channels, one for transmit, and one for receive
 */
static int dmatest_slave_func(struct dma_chan *tx_chan, struct dma_chan *rx_chan)
{
    unsigned int src_off, dst_off, len;
    unsigned int error_count;
    unsigned int failed_tests = 0;
    unsigned int total_tests = 0;
    dma_cookie_t tx_cookie;
    dma_cookie_t rx_cookie;
    enum dma_status status;
    enum dma_ctrl_flags flags;
    int ret;
    int src_cnt;
    int dst_cnt;
    int bd_cnt = 5;
    int i;
    u8 **srcs;
    u8 **dsts;

    ret = -ENOMEM;

    smp_rmb();
    src_cnt = dst_cnt = bd_cnt;

    srcs = kcalloc(src_cnt + 1, sizeof(u8 *), GFP_KERNEL);
    if (!srcs)
        goto err_srcs;
    for (i = 0; i < src_cnt; i++)
    {
        srcs[i] = kmalloc(test_buf_size, GFP_KERNEL);
        if (!srcs[i])
            goto err_srcbuf;
    }
    srcs[i] = NULL;

    dsts = kcalloc(dst_cnt + 1, sizeof(u8 *), GFP_KERNEL);
    if (!dsts)
        goto err_dsts;
    for (i = 0; i < dst_cnt; i++)
    {
        dsts[i] = kmalloc(test_buf_size, GFP_KERNEL);
        if (!dsts[i])
            goto err_dstbuf;
    }
    dsts[i] = NULL;

    set_user_nice(current, 10);

    flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

    while (!(iterations && total_tests >= iterations))
    {
        struct dma_device *tx_dev = tx_chan->device;
        struct dma_device *rx_dev = rx_chan->device;
        struct dma_async_tx_descriptor *txd = NULL;
        struct dma_async_tx_descriptor *rxd = NULL;
        dma_addr_t dma_srcs[src_cnt];
        dma_addr_t dma_dsts[dst_cnt];
        struct completion rx_cmp;
        struct completion tx_cmp;
        unsigned long rx_tmo = msecs_to_jiffies(300000); /* RX takes longer */
        unsigned long tx_tmo = msecs_to_jiffies(30000);
        u8 align = 0;
        struct scatterlist tx_sg[bd_cnt];
        struct scatterlist rx_sg[bd_cnt];

        total_tests++;

        /* honor larger alignment restrictions */
        align = tx_dev->copy_align;
        if (rx_dev->copy_align > align)
            align = rx_dev->copy_align;

        if (1 << align > test_buf_size)
        {
            pr_err("%u-byte buffer too small for %d-byte alignment\n",
                   test_buf_size, 1 << align);
            break;
        }

        len = dmatest_random() % test_buf_size + 1;
        len = (len >> align) << align;
        if (!len)
            len = 1 << align;
        src_off = dmatest_random() % (test_buf_size - len + 1);
        dst_off = dmatest_random() % (test_buf_size - len + 1);

        src_off = (src_off >> align) << align;
        dst_off = (dst_off >> align) << align;

        dmatest_init_srcs(srcs, src_off, len);
        dmatest_init_dsts(dsts, dst_off, len);

        for (i = 0; i < src_cnt; i++)
        {
            u8 *buf = srcs[i] + src_off;

            dma_srcs[i] = dma_map_single(tx_dev->dev, buf, len,
                                         DMA_MEM_TO_DEV);
        }

        for (i = 0; i < dst_cnt; i++)
        {
            dma_dsts[i] = dma_map_single(rx_dev->dev,
                                         dsts[i],
                                         test_buf_size,
                                         DMA_MEM_TO_DEV);

            dma_unmap_single(rx_dev->dev, dma_dsts[i],
                             test_buf_size,
                             DMA_MEM_TO_DEV);

            dma_dsts[i] = dma_map_single(rx_dev->dev,
                                         dsts[i],
                                         test_buf_size,
                                         DMA_DEV_TO_MEM);
        }

        sg_init_table(tx_sg, bd_cnt);
        sg_init_table(rx_sg, bd_cnt);

        for (i = 0; i < bd_cnt; i++)
        {
            sg_dma_address(&tx_sg[i]) = dma_srcs[i];
            sg_dma_address(&rx_sg[i]) = dma_dsts[i] + dst_off;

            sg_dma_len(&tx_sg[i]) = len;
            sg_dma_len(&rx_sg[i]) = len;
        }

        rxd = rx_dev->device_prep_slave_sg(rx_chan, rx_sg, bd_cnt,
                                           DMA_DEV_TO_MEM, flags, NULL);

        txd = tx_dev->device_prep_slave_sg(tx_chan, tx_sg, bd_cnt,
                                           DMA_MEM_TO_DEV, flags, NULL);

        if (!rxd || !txd)
        {
            for (i = 0; i < src_cnt; i++)
                dma_unmap_single(tx_dev->dev, dma_srcs[i], len,
                                 DMA_MEM_TO_DEV);
            for (i = 0; i < dst_cnt; i++)
                dma_unmap_single(rx_dev->dev, dma_dsts[i],
                                 test_buf_size,
                                 DMA_DEV_TO_MEM);
            pr_warn(
                "%s: #%u: prep error with src_off=0x%x ",
                "thread_name", total_tests - 1, src_off);
            pr_warn("dst_off=0x%x len=0x%x\n",
                    dst_off, len);
            msleep(100);
            failed_tests++;
            continue;
        }

        init_completion(&rx_cmp);
        rxd->callback = dmatest_slave_rx_callback;
        rxd->callback_param = &rx_cmp;
        rx_cookie = rxd->tx_submit(rxd);

        init_completion(&tx_cmp);
        txd->callback = dmatest_slave_tx_callback;
        txd->callback_param = &tx_cmp;
        tx_cookie = txd->tx_submit(txd);

        if (dma_submit_error(rx_cookie) ||
            dma_submit_error(tx_cookie))
        {
            pr_warn(
                "%s: #%u: submit error %d/%d with src_off=0x%x ",
                "thread_name", total_tests - 1,
                rx_cookie, tx_cookie, src_off);
            pr_warn("dst_off=0x%x len=0x%x\n",
                    dst_off, len);
            msleep(100);
            failed_tests++;
            continue;
        }
        dma_async_issue_pending(tx_chan);
        dma_async_issue_pending(rx_chan);

        tx_tmo = wait_for_completion_timeout(&tx_cmp, tx_tmo);

        status = dma_async_is_tx_complete(tx_chan, tx_cookie,
                                          NULL, NULL);

        if (tx_tmo == 0)
        {
            pr_warn("%s: #%u: tx test timed out\n",
                    "thread_name", total_tests - 1);
            failed_tests++;
            continue;
        }
        else if (status != DMA_COMPLETE)
        {
            pr_warn(
                "%s: #%u: tx got completion callback, ",
                "thread_name", total_tests - 1);
            pr_warn("but status is \'%s\'\n",
                    status == DMA_ERROR ? "error" : "in progress");
            failed_tests++;
            continue;
        }

        rx_tmo = wait_for_completion_timeout(&rx_cmp, rx_tmo);
        status = dma_async_is_tx_complete(rx_chan, rx_cookie,
                                          NULL, NULL);

        if (rx_tmo == 0)
        {
            pr_warn("%s: #%u: rx test timed out\n",
                    "thread_name", total_tests - 1);
            failed_tests++;
            continue;
        }
        else if (status != DMA_COMPLETE)
        {
            pr_warn(
                "%s: #%u: rx got completion callback, ",
                "thread_name", total_tests - 1);
            pr_warn("but status is \'%s\'\n",
                    status == DMA_ERROR ? "error" : "in progress");
            failed_tests++;
            continue;
        }

        /* Unmap by myself */
        for (i = 0; i < dst_cnt; i++)
            dma_unmap_single(rx_dev->dev, dma_dsts[i],
                             test_buf_size, DMA_DEV_TO_MEM);

        error_count = 0;

        pr_debug("%s: verifying source buffer...\n", "thread_name");
        error_count += dmatest_verify(srcs, 0, src_off,
                                      0, PATTERN_SRC, true);
        error_count += dmatest_verify(srcs, src_off,
                                      src_off + len, src_off,
                                      PATTERN_SRC | PATTERN_COPY, true);
        error_count += dmatest_verify(srcs, src_off + len,
                                      test_buf_size, src_off + len,
                                      PATTERN_SRC, true);

        pr_debug("%s: verifying dest buffer...\n",
                 "thread->task->comm");
        error_count += dmatest_verify(dsts, 0, dst_off,
                                      0, PATTERN_DST, false);
        error_count += dmatest_verify(dsts, dst_off,
                                      dst_off + len, src_off,
                                      PATTERN_SRC | PATTERN_COPY, false);
        error_count += dmatest_verify(dsts, dst_off + len,
                                      test_buf_size, dst_off + len,
                                      PATTERN_DST, false);

        if (error_count)
        {
            pr_warn("%s: #%u: %u errors with ",
                    "thread_name", total_tests - 1, error_count);
            pr_warn("src_off=0x%x dst_off=0x%x len=0x%x\n",
                    src_off, dst_off, len);
            failed_tests++;
        }
        else
        {
            pr_debug("%s: #%u: No errors with ",
                     "thread_name", total_tests - 1);
            pr_debug("src_off=0x%x dst_off=0x%x len=0x%x\n",
                     src_off, dst_off, len);
        }
    }

    ret = 0;
    for (i = 0; dsts[i]; i++)
        kfree(dsts[i]);
err_dstbuf:
    kfree(dsts);
err_dsts:
    for (i = 0; srcs[i]; i++)
        kfree(srcs[i]);
err_srcbuf:
    kfree(srcs);
err_srcs:
    pr_notice("%s: terminating after %u tests, %u failures (status %d)\n",
              "thread_name", total_tests, failed_tests, ret);

    return ret;
}

static int xilinx_axidmatest_probe(struct platform_device *pdev)
{
    int err;

    chan = dma_request_slave_channel(&pdev->dev, "axidma0");
    if (IS_ERR(chan))
    {
        pr_err("xilinx_dmatest: No Tx channel\n");
        return PTR_ERR(chan);
    }

    rx_chan = dma_request_slave_channel(&pdev->dev, "axidma1");
    if (IS_ERR(rx_chan))
    {
        err = PTR_ERR(rx_chan);
        pr_err("xilinx_dmatest: No Rx channel\n");
        goto free_tx;
    }

    err = dmatest_slave_func(chan, rx_chan);
    if (err)
    {
        pr_err("xilinx_dmatest: Unable to add channels\n");
        goto free_rx;
    }

    return 0;

free_rx:
    dma_release_channel(rx_chan);
free_tx:
    dma_release_channel(chan);

    return err;
}

static int xilinx_axidmatest_remove(struct platform_device *pdev)
{
    pr_info("xilinx_dmatest: dropped channel %s\n", dma_chan_name(chan));
    dmaengine_terminate_all(chan);
    dma_release_channel(chan);
    pr_info("xilinx_dmatest: dropped channel %s\n", dma_chan_name(rx_chan));
    dmaengine_terminate_all(rx_chan);
    dma_release_channel(rx_chan);

    return 0;
}

static const struct of_device_id xilinx_axidmatest_of_ids[] = {
    {
        .compatible = "mumu,axi-dma-loop",
    },
    {}};

static struct platform_driver xilinx_axidmatest_driver = {
    .driver = {
        .name = "axidmaloop",
        .owner = THIS_MODULE,
        .of_match_table = xilinx_axidmatest_of_ids,
    },
    .probe = xilinx_axidmatest_probe,
    .remove = xilinx_axidmatest_remove,
};

static int __init axidma_init(void)
{
    return platform_driver_register(&xilinx_axidmatest_driver);
}
late_initcall(axidma_init);

static void __exit axidma_exit(void)
{
    platform_driver_unregister(&xilinx_axidmatest_driver);
}
module_exit(axidma_exit)

    MODULE_AUTHOR("mumu");
MODULE_DESCRIPTION("Xilinx AXI DMA Loop Test");
MODULE_LICENSE("GPL v2");
