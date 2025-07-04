#!/bin/bash

set -ex

BS_DIR_TOP=$(cd `dirname $0` ; pwd)

BS_DIR_TOOLCHAIN_ARM64=../gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-

# BS_CONFIG_KERNEL=rockchip_linux_defconfig
BS_CONFIG_KERNEL=dc-a588_defconfig
BS_CONFIG_KERNEL_DTB=rk3588-evb7-lp4-v10-linux.img


make ARCH=arm64 ${BS_CONFIG_KERNEL} CROSS_COMPILE=${BS_DIR_TOOLCHAIN_ARM64}
make ARCH=arm64 ${BS_CONFIG_KERNEL_DTB} CROSS_COMPILE=${BS_DIR_TOOLCHAIN_ARM64} -j`nproc`


exit 0

