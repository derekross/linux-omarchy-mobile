#!/bin/bash
# usage: run-case.sh NAME "GNSS_SPI_ARGS"  (palmtop freshly booted)
cd ~/git/omarchy-mobile
until tools/phone-ssh --fresh true 2>/dev/null; do sleep 5; done
tools/phone-ssh "until [ \$(cut -d. -f1 /proc/uptime) -ge 60 ]; do sleep 2; done; systemd-run -q --unit=x-g7 -E GNSS_SPI_ARGS='$2' sh -c '/usr/local/bin/g7-strace > /var/log/omamobile/g7-run.txt 2>&1'; sleep 30; dmesg | grep -iE 'feedback delay|cs_setup|usi3|hs:|realign'; while systemctl is-active -q x-g7; do sleep 10; done; L=/var/log/omamobile/g7/modem.log; for p in 'BAD checksum' 'ACK timeout' 'soft_dog pos' HARDFAULT 'RX RESET-ACK' 'RX packet len'; do printf '%s: ' \"\$p\"; grep -ac \"\$p\" \$L; done; grep -aiE 'svid|sv_id|C/N0|cn0' \$L | tail -3; grep -ao 'BAD checksum: footer=0x[0-9a-f]*, calc=0x[0-9a-f]*, len=[0-9]*' \$L; mv /var/log/omamobile/g7 /var/log/omamobile/g7-$1"
