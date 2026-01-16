#!/bin/sh

while true; do
    val=$(devmem 0x40000084 32)

    if [ "$val" = "0x00000001" ]; then
        echo "reboot flag detected"

        # 清标志，防止无限重启
        devmem 0x40000084 32 0

        sync
        reboot
    fi

    sleep 1
done
