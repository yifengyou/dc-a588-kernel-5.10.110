#!/bin/bash

set -x
JOB=`sed -n "N;/processor/p" /proc/cpuinfo|wc -l`
WORKDIR=`pwd`
ARCH=`uname -m`

export KERNEL_TARGET=dc-a588

if [ X"${ARCH}" == X"aarch64" ] ; then
	GCC=""
	CROSS_COMPILE_ARM64=""
elif [ X"${ARCH}" == X"x86_64" ] ; then
	GCC=`realpath ../gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu`
	CROSS_COMPILE_ARM64=${GCC}/bin/aarch64-none-linux-gnu-
	echo "using gcc: [${CROSS_COMPILE_ARM64}]"
else
	echo "${ARCH} is not supported now!"
	exit 1
fi


# clean
# make ARCH=arm64 CROSS_COMPILE=$CROSS_COMPILE_ARM64 mrproper

# kernel
if [ -f .config ] ; then
	cp -a .config .config-bak
fi
make ARCH=arm64 CROSS_COMPILE=$CROSS_COMPILE_ARM64 ${KERNEL_TARGET}_defconfig
if [ $? -ne 0 ] ; then
	echo "config failed!"
	exit 1
fi

if [ -f .config-bak ] ; then
	diff .config .config-bak
	if [ $? -eq 0 ] ; then
		cp -a .config-bak .config
	fi
fi

set -e
make ARCH=arm64 CROSS_COMPILE=$CROSS_COMPILE_ARM64 -j$JOB
# make ARCH=arm64 CROSS_COMPILE=$CROSS_COMPILE_ARM64 dtbs -j$JOB
# make ARCH=arm64 CROSS_COMPILE=$CROSS_COMPILE_ARM64 ${KERNEL_TARGET}.img

rm -rf output
mkdir -p output/modules
cp arch/arm64/boot/Image output

cd output
cp -a ../tools/boot/* .
cp -a ../arch/arm64/boot/dts/rockchip/rk3588-dc-a588.dtb .
../tools/mkimage -f boot.its -E -p 0x800 boot.img

ls -alh boot.img
md5sum  boot.img

cd ${WORKDIR}
cat ./include/config/kernel.release
echo "All done!"

exit 0

# origin build script backup

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

