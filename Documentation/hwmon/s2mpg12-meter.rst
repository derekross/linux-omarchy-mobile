.. SPDX-License-Identifier: GPL-2.0-only

Kernel driver s2mpg12-meter
===========================

Supported chips:

  * Samsung S2MPG12 (main PMIC of Google's gs201)

    Prefix: 's2mpg12'

  * Samsung S2MPG13 (sub PMIC of Google's gs201)

    Prefix: 's2mpg13'

Description
-----------

Each PMIC has twelve power meter channels. The device tree assigns each
channel to one of the PMIC's bucks or LDOs, or to an external shunt
resistor, and names it. The PMIC samples every channel at 125 Hz and adds
up the power in an accumulator; the driver latches and clears the
accumulators when a value is read and at least ``update_interval``
milliseconds have passed since the last latch, and every 30 minutes so
that the sample counter never saturates.

The meter can get stuck, with a saturated sample count and latched values
that no longer change; turning it off and on does not help. The driver
therefore resets it (the software reset bit in the PMIC's meter trim
block) before configuring it at probe, and resets and reconfigures it
whenever a latch returns a saturated count or more samples than the time
since the previous latch allows. That window is dropped: the energy
counters skip it and the power averages keep their previous value.

Sysfs entries
-------------

======================= ==================================================
update_interval         Minimum time between two latches, in ms (RW,
                        100 to 60000, default 1000)
power[1-12]_average     Average power of the channel over the last latch
                        window, in microwatts
power[1-12]_label       Rail name from the device tree
energy[1-12]_input      Energy of the channel since the driver was
                        loaded, in microjoules
energy[1-12]_label      Rail name from the device tree
======================= ==================================================

Only the channels the device tree describes are shown.
