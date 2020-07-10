# modpl360
Linux device driver for PL360

## Build

    sudo -s
    apt install raspberrypi-kernel-headers wpan-tools
    make
    make install install raspberrypi-kernel-headers
    make dts
    
## Setup

Add to /boot/config.txt with your pin numbers:

    dtoverlay=pl360,ldo_pin=17,nrst_pin=19,irq_pin=12
    
If you use SPI other tan spi0.0, change pl360-overlay.dts and run'make dts' again

## Network setup

    iwpan dev wpan0 set pan_id 0x777
    iwpan phy phy0 set channel 0 11
    iwpan dev wpan0 set ackreq_default 1
    ifconfig wpan0 up 
    ip link add link wpan0 name lowpan0 type lowpan
    ip route add 2001::/64 dev lowpan0
    ip addr add  2001::4/128 dev lowpan0
    ifconfig lowpan0 up
    
Check PLC acces to remote node:

    ping6 -i0.1 2001::3

## Wireshark

Command to observe packets:

    plink.exe -batch -ssh -pw pass user@host "sudo /usr/sbin/tcpdump -Unqi wpan0 -s 0 -w -"   | "C:\Program Files\Wireshark\Wireshark.exe" -k -i -
    
