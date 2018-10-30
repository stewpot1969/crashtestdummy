target extended-remote /dev/ttyACM1
monitor version
monitor swdp_scan
attach 1
load main.elf
