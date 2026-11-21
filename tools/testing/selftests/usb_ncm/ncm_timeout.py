#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Compile the actual NCM callbacks with host stubs; no USB device needed."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

root = Path(os.environ.get('KERNEL_SRC', Path(__file__).resolve().parents[4]))
source_file = root / 'drivers/usb/gadget/function/f_ncm.c'
if not source_file.is_file():
    print('SKIP: kernel sources unavailable; set KERNEL_SRC to the source tree')
    raise SystemExit(4)
source = source_file.read_text()
cc = os.environ.get('HOSTCC', 'cc')
if shutil.which(cc) is None:
    print('SKIP: host C compiler required')
    raise SystemExit(4)


def function(signature):
    return signature + source.split(signature, 1)[1].split('\n}', 1)[0] + '\n}\n'


stubs = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, v) ((x) = (v))
#define container_of(p, t, m) ((t *)((char *)(p) - offsetof(t, m)))
#define NETDEV_TX_BUSY 16
#define TX_TIMEOUT_NSECS 300000
#define HRTIMER_MODE_REL_SOFT 0
enum hrtimer_restart { HRTIMER_NORESTART, HRTIMER_RESTART };
struct hrtimer { long forwarded; int armed; };
struct net_device;
struct net_device_ops { int (*ndo_start_xmit)(void *, struct net_device *); };
struct net_device { const struct net_device_ops *netdev_ops; };
struct usb_function { int unused; };
struct f_ncm {
    struct { struct usb_function func; bool is_suspend; } port;
    struct hrtimer task_timer;
    struct net_device *netdev;
    bool suspended;
};
#define func_to_ncm(f) container_of(f, struct f_ncm, port.func)
static struct f_ncm *current;
static int result, attempts, suspend_in_xmit;
static int transmit(void *skb, struct net_device *dev)
{
    (void)skb; (void)dev; attempts++;
    if (suspend_in_xmit) current->suspended = true;
    return result;
}
static void hrtimer_forward_now(struct hrtimer *t, long ns) { t->forwarded += ns; }
static int hrtimer_try_to_cancel(struct hrtimer *t) { t->armed = 0; return 0; }
static void hrtimer_start(struct hrtimer *t, long ns, int mode)
{ (void)ns; (void)mode; t->armed = 1; }
'''
main = r'''
int main(void)
{
    const struct net_device_ops ops = { .ndo_start_xmit = transmit };
    struct net_device dev = { .netdev_ops = &ops };
    struct f_ncm ncm = { .netdev = &dev };
    current = &ncm;
    result = 0;
    assert(ncm_tx_timeout(&ncm.task_timer) == HRTIMER_NORESTART);
    assert(attempts == 1 && ncm.task_timer.forwarded == 0);
    result = NETDEV_TX_BUSY; attempts = 0;
    for (int i = 0; i < 10; i++) {
        assert(ncm_tx_timeout(&ncm.task_timer) == HRTIMER_RESTART);
        assert(attempts == i + 1);
        assert(ncm.task_timer.forwarded == (i + 1) * TX_TIMEOUT_NSECS);
    }
    result = 0;
    assert(ncm_tx_timeout(&ncm.task_timer) == HRTIMER_NORESTART);
    ncm_suspend(&ncm.port.func); attempts = 0;
    assert(ncm.suspended && !ncm.task_timer.armed);
    for (int i = 0; i < 100; i++)
        assert(ncm_tx_timeout(&ncm.task_timer) == HRTIMER_NORESTART);
    assert(attempts == 0);
    ncm_resume(&ncm.port.func);
    assert(!ncm.suspended && ncm.task_timer.armed);
    assert(ncm_tx_timeout(&ncm.task_timer) == HRTIMER_NORESTART && attempts == 1);
    ncm.port.is_suspend = true; attempts = 0;
    assert(ncm_tx_timeout(&ncm.task_timer) == HRTIMER_NORESTART && attempts == 0);
    ncm.port.is_suspend = false; suspend_in_xmit = 1; result = NETDEV_TX_BUSY;
    assert(ncm_tx_timeout(&ncm.task_timer) == HRTIMER_NORESTART && attempts == 1);
    suspend_in_xmit = 0; ncm.suspended = false; ncm.netdev = NULL; attempts = 0;
    assert(ncm_tx_timeout(&ncm.task_timer) == HRTIMER_NORESTART && attempts == 0);
    ncm.task_timer.armed = 0; ncm_resume(&ncm.port.func);
    assert(!ncm.task_timer.armed);
    puts("ok - busy retries, success, suspend, resume and disconnect");
    return 0;
}
'''
callbacks = ''.join(function(s) for s in (
    'static enum hrtimer_restart ncm_tx_timeout(struct hrtimer *data)',
    'static void ncm_suspend(struct usb_function *f)',
    'static void ncm_resume(struct usb_function *f)'))
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    (p / 'test.c').write_text(stubs + callbacks + main)
    subprocess.run([cc, '-Wall', '-Wextra', '-Werror', str(p / 'test.c'), '-o', str(p / 'test')], check=True)
    subprocess.run([str(p / 'test')], check=True)
